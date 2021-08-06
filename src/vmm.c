/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#include <autoconf.h>

#include <stdio.h>
#include <assert.h>
#include <string.h>

#include <allocman/bootstrap.h>
#include <allocman/vka.h>
#include <elf/elf.h>
#include <vka/capops.h>
#include <vka/object.h>

#include <vspace/vspace.h>
#include <simple/simple.h>
#include <simple-default/simple-default.h>
#include <platsupport/io.h>
#include <sel4platsupport/platsupport.h>
#include <sel4platsupport/io.h>
#include <sel4utils/vspace.h>
#include <sel4utils/vspace_internal.h>
#include <sel4utils/api.h>
#include <sel4utils/thread.h>
#include <utils/util.h>
#include <cpio/cpio.h>


static int vm_copyin_lock = 0;
static int vm_map_device_lock = 0;

#if CONFIG_MAX_NUM_NODES > 1
static inline void lock(int *lock)
{
    int tmp = 0;
    while (!__atomic_compare_exchange_n(lock, &tmp, 1, false, __ATOMIC_ACQUIRE, __ATOMIC_ACQUIRE)) {
        tmp = 0;
    }
}

static inline void unlock(int *lock)
{
    __atomic_store_n(lock, 0, __ATOMIC_RELEASE);
}
#else
#define lock(x)
#define unlock(x)
#endif

#if CONFIG_MAX_NUM_NODES > 1
#define CONFIG_PER_VCPU_VMM
#endif
/* IRQs are directly injected from the irq handler thread */
#define CONFIG_DIRECT_VCPU_IRQ

/* RISCV instruction decoding */

/* format for store instructions  
 * 31 -------------------------- 0
 * immi[15:5] | rs2 (5) | rs1 (5) | func3 (3) | immi[4:0] | opcode (7)
 * rs1 is for memory address.
 * rs2 s the data to be stored.
 */


#define FUNC3_MASK  0x7
#define FUNC3_SHIFT 12 
#define FUNC3(x)    ((x >> FUNC3_SHIFT) & FUNC3_MASK)

#define OP_MASK     0x7f
#define OPCODE(x)   (x & OP_MASK)

#define RMASK       0x1f

#define LD_OP       0b0000011
#define LD_RD(x)    ((x >> 7) & RMASK)
#define LD_RS1(x)   ((x >> 15) & RMASK);
#define LD_IMMI(x)  ((x >> 20) & (BIT(13) - 1))


#define ST_OP       0b0100011
#define RS2_SHIFT   20
#define RS1_SHIFT   15
#define ST_RS2(x)   ((x >> RS2_SHIFT) & RMASK)
#define ST_RS1(x)   ((x >> RS1_SHIFT) & RMASK)

#define LD_ST_B        0
#define LD_ST_H        1
#define LD_ST_W        2
#define LD_ST_D        3

typedef struct riscv_inst {
    int opcode;
    int rd;
    int rs1;
    int rs2;
    int immi;
    int func3;
    int func5;
    uint32_t inst;
} riscv_inst_t;

/* interrupt server stuff */
typedef int irq_t;

struct irq_data;

/**
 * @return non-zero if the IRQ should be ACKed
 */
typedef void (*irq_handler_fn)(struct irq_data* irq);

struct irq_data {
/// irq number
    irq_t irq;
/// The capability for this irq number
    seL4_CPtr cap;
/// Client data: function to call when the IRQ arrives
    irq_handler_fn cb;
/// Client specific handle to pass to the callback function
    void* token;
};

/**
 * Allows a client to acknowledge an IRQ
 * @param[in] irq  The IRQ to acknowledge
 */
void irq_data_ack_irq(struct irq_data* irq);

/*********************************
 *** IRQ server node functions ***
 *********************************/

typedef struct irq_server_node* irq_server_node_t;


#define NIRQS_PER_NODE        seL4_BadgeBits

/*************************
 *** Generic functions ***
 *************************/

void irq_data_ack_irq(struct irq_data* irq)
{
    if (irq == NULL || irq->cap == seL4_CapNull) {
        ZF_LOGE("IRQ data invalid when acknowledging IRQ\n");
    } else {
        seL4_IRQHandler_Ack(irq->cap);
    }
}

/***********************
 *** IRQ server node ***
 ***********************/

struct irq_server_node {
/// Information about the IRQ that is assigned to each badge bit
    struct irq_data irqs[NIRQS_PER_NODE];
/// The notification object that IRQs arrive on
    seL4_CPtr notification;
/// A mask for the badge. All set bits within the badge are treated as reserved.
    seL4_Word badge_mask;
};

/* Executes the registered callback for incoming IRQS */
static void irq_server_node_handle_irq(struct irq_server_node *n, seL4_Word badge)
{
    struct irq_data *irqs;
    irqs = n->irqs;
    /* Mask out reserved bits */
    badge = badge & n->badge_mask;
    /* For each bit, call the registered handler */
    while (badge) {
        int irq_idx;
        struct irq_data *irq;
        irq_idx = CTZL(badge);
        irq = &irqs[irq_idx];
        irq->cb(irq);
        badge &= ~BIT(irq_idx);
    }
}

/* Binds and IRQ to an endpoint */
static seL4_CPtr irq_bind(irq_t irq, seL4_CPtr notification_cap, int idx, vka_t *vka, simple_t *simple)
{
    seL4_CPtr irq_cap, bnotification_cap;
    cspacepath_t irq_path, notification_path, bnotification_path;
    seL4_Word badge;
    int err;

    /* Create an IRQ cap */
    err = vka_cspace_alloc(vka, &irq_cap);
    if (err != 0) {
        ZF_LOGE("Failed to allocate cslot for irq\n");
        return seL4_CapNull;
    }
    vka_cspace_make_path(vka, irq_cap, &irq_path);
    err = simple_get_IRQ_handler(simple, irq, irq_path);
    if (err != seL4_NoError) {
        ZF_LOGE("Failed to get cap to irq_number %d\n", irq);
        vka_cspace_free(vka, irq_cap);
        return seL4_CapNull;
    }
    /* Badge the provided endpoint. The bit position of the badge tells us the array
     * index of the associated IRQ data. */
    err = vka_cspace_alloc(vka, &bnotification_cap);
    if (err != 0) {
        ZF_LOGE("Failed to allocate cslot for irq\n");
        vka_cspace_free(vka, irq_cap);
        return seL4_CapNull;
    }
    vka_cspace_make_path(vka, notification_cap, &notification_path);
    vka_cspace_make_path(vka, bnotification_cap, &bnotification_path);
    badge = BIT(idx);
    err = vka_cnode_mint(&bnotification_path, &notification_path, seL4_AllRights, badge);
    if (err != seL4_NoError) {
        ZF_LOGE("Failed to badge IRQ notification endpoint\n");
        vka_cspace_free(vka, irq_cap);
        vka_cspace_free(vka, bnotification_cap);
        return seL4_CapNull;
    }
    /* bind the IRQ cap to our badged endpoint */
    err = seL4_IRQHandler_SetNotification(irq_cap, bnotification_cap);
    if (err != seL4_NoError) {
        ZF_LOGE("Failed to bind IRQ handler to notification\n");
        vka_cspace_free(vka, irq_cap);
        vka_cspace_free(vka, bnotification_cap);
        return seL4_CapNull;
    }
    /* Finally ACK any pending IRQ and enable the IRQ */
    seL4_IRQHandler_Ack(irq_cap);

    ZF_LOGD("Registered IRQ %d with badge 0x%lx\n", irq, BIT(idx));
    return irq_cap;
}

/* Registers an IRQ callback and enabled the IRQ */
struct irq_data *irq_server_node_register_irq(irq_server_node_t n, irq_t irq, irq_handler_fn cb,
                             void *token, vka_t *vka, seL4_CPtr cspace,
                             simple_t *simple) {
    struct irq_data *irqs;
    int i;
    irqs = n->irqs;

    for (i = 0; i < NIRQS_PER_NODE; i++) {
        /* If a cap has not been registered and the bit in the mask is not set */
        if (irqs[i].cap == seL4_CapNull && (n->badge_mask & BIT(i))) {
            irqs[i].cap = irq_bind(irq, n->notification, i, vka, simple);
            if (irqs[i].cap == seL4_CapNull) {
                ZF_LOGD("Failed to bind IRQ\n");
                return NULL;
            }
            irqs[i].irq = irq;
            irqs[i].cb = cb;
            irqs[i].token = token;
            return &irqs[i];
        }
    }
    return NULL;
}

/* Creates a new IRQ server node which contains Thread data and registered IRQ data. */
struct irq_server_node *irq_server_node_new(seL4_CPtr notification, seL4_Word badge_mask) {
    struct irq_server_node *n;
    n = calloc(1, sizeof(*n));
    if (n) {
        n->notification = notification;
        n->badge_mask = badge_mask;
    }
    return n;
}

/*************************
 *** IRQ server thread ***
 *************************/

struct irq_server_thread {
/// IRQ data which this thread is responsible for
    struct irq_server_node *node;
/// A synchronous endpoint to deliver IRQ messages to.
    seL4_CPtr delivery_sep;
/// The label that should be assigned to outgoing synchronous messages.
    seL4_Word label;
/// Thread data
    sel4utils_thread_t thread;
/// notification object data
    vka_object_t notification;
/// Linked list chain
    struct irq_server_thread *next;
};

/* IRQ handler thread. Wait on a notification object for IRQs. When one arrives, send a
 * synchronous message to the registered endpoint. If no synchronous endpoint was
 * registered, call the appropriate handler function directly (must be thread safe) */
static void _irq_thread_entry(struct irq_server_thread *st)
{
    seL4_CPtr sep;
    seL4_CPtr notification;
    uintptr_t node_ptr;
    seL4_Word label;

    sep = st->delivery_sep;
    notification = st->node->notification;
    node_ptr = (uintptr_t)st->node;
    label = st->label;
    ZF_LOGD("thread started. Waiting on endpoint %d\n", notification);

    while (1) {
        seL4_Word badge;
        seL4_Wait(notification, &badge);
        assert(badge != 0);
        if (sep != seL4_CapNull) {
            /* Synchronous endpoint registered. Send IPC */
            seL4_MessageInfo_t info = seL4_MessageInfo_new(label, 0, 0, 2);
            seL4_SetMR(0, badge);
            seL4_SetMR(1, node_ptr);
            seL4_Send(sep, info);
        } else {
            /* No synchronous endpoint. Call the handler directly */
            irq_server_node_handle_irq(st->node, badge);
        }
    }
}

/* Creates a new thread for an IRQ server */
struct irq_server_thread *irq_server_thread_new(vspace_t *vspace, vka_t *vka, seL4_CPtr cspace,
                                                seL4_Word priority, simple_t *simple,
                                                seL4_Word label, seL4_CPtr sep)
{
    struct irq_server_thread *st;
    int err;

    /* Allocate memory for the structure */
    st = malloc(sizeof(*st));
    if (st == NULL) {
        return NULL;
    }
    st->node = irq_server_node_new(0, MASK(NIRQS_PER_NODE));
    if (st->node == NULL) {
        free(st);
        return NULL;
    }

    /* Initialise structure */
    st->delivery_sep = sep;
    st->label = label;
    st->next = NULL;
    /* Create an endpoint to listen on */
    err = vka_alloc_notification(vka, &st->notification);
    if (err) {
        ZF_LOGE("Failed to allocate IRQ notification endpoint for IRQ server thread\n");
        return NULL;
    }
    st->node->notification = st->notification.cptr;
    /* Create the IRQ thread */
    sel4utils_thread_config_t config = thread_config_default(simple, cspace, seL4_NilData, 0, priority);
    err = sel4utils_configure_thread_config(vka, vspace, vspace, config, &st->thread);

    if (err) {
        ZF_LOGE("Failed to configure IRQ server thread\n");
        return NULL;
    }
    /* Start the thread */
    err = sel4utils_start_thread(&st->thread, (void *)_irq_thread_entry, st, NULL, 1);
    seL4_DebugNameThread(st->thread.tcb.cptr, "irqserver");
    if (err) {
        ZF_LOGE("Failed to start IRQ server thread\n");
        return NULL;
    }
    return st;
}

/******************
 *** IRQ server ***
 ******************/

struct irq_server {
    seL4_CPtr delivery_ep;
    vka_object_t reply;
    seL4_Word label;
    int max_irqs;
    vspace_t *vspace;
    seL4_CPtr cspace;
    vka_t* vka;
    seL4_Word thread_priority;
    simple_t simple;
    struct irq_server_thread *server_threads;
    void *vm;
    /* affinity of the server threads */
    int affinity;
};

typedef struct irq_server *irq_server_t;

/* Handle an incoming IPC from a server node */
void irq_server_handle_irq_ipc(irq_server_t irq_server UNUSED)
{
    seL4_Word badge;
    uintptr_t node_ptr;

    badge = seL4_GetMR(0);
    node_ptr = seL4_GetMR(1);
    if (node_ptr == 0) {
        ZF_LOGE("Invalid data in irq server IPC\n");
    } else {
        irq_server_node_handle_irq((struct irq_server_node *)node_ptr, badge);
    }
}

/* Register for a function to be called when an IRQ arrives */
struct irq_data *irq_server_register_irq(irq_server_t irq_server, irq_t irq,
                                         irq_handler_fn cb, void *token)
{
    struct irq_server_thread *st;
    struct irq_data *irq_data;

    /* Try to assign the IRQ to an existing node */
    for (st = irq_server->server_threads; st != NULL; st = st->next) {
        irq_data = irq_server_node_register_irq(st->node, irq, cb, token,
                                                irq_server->vka, irq_server->cspace,
                                                &irq_server->simple);
        if (irq_data) {
            return irq_data;
        }
    }
    /* Try to create a new node */
    if (st == NULL && irq_server->max_irqs < 0) {
        /* Create the node */
        ZF_LOGD("Spawning new IRQ server thread\n");
        st = irq_server_thread_new(irq_server->vspace, irq_server->vka, irq_server->cspace,
                                   irq_server->thread_priority, &irq_server->simple,
                                   irq_server->label, irq_server->delivery_ep);
#if CONFIG_MAX_NUM_NODES > 1
        seL4_TCB_SetAffinity(st->thread.tcb.cptr, irq_server->affinity);
#endif
        if (st == NULL) {
            ZF_LOGE("Failed to create server thread\n");
            return NULL;
        }

        st->next = irq_server->server_threads;
        irq_server->server_threads = st;
        irq_data = irq_server_node_register_irq(st->node, irq, cb, token,
                                                irq_server->vka, irq_server->cspace,
                                                &irq_server->simple);
        if (irq_data) {
            return irq_data;
        }
    }
    /* Give up */
    ZF_LOGD("Failed to register for IRQ %d\n", irq);
    return NULL;
}

/* Create a new IRQ server */
int irq_server_new(vspace_t *vspace, vka_t *vka, seL4_CPtr cspace, seL4_Word priority,
               simple_t *simple, seL4_CPtr sync_ep, seL4_Word label,
               int nirqs, int affinity, irq_server_t *ret_irq_server)
{
    struct irq_server *irq_server;

    /* Structure allocation and initialisation */
    irq_server = malloc(sizeof(*irq_server));
    if (irq_server == NULL) {
        ZF_LOGE("malloc failed on irq server memory allocation");
        return -1;
    }

    if (config_set(CONFIG_KERNEL_RT) && vka_alloc_reply(vka, &irq_server->reply) != 0) {
        ZF_LOGE("Failed to allocate reply object");
        free(irq_server);
        return -1;
    }

    irq_server->delivery_ep = sync_ep;
    irq_server->label = label;
    irq_server->max_irqs = nirqs;
    irq_server->vspace = vspace;
    irq_server->cspace = cspace;
    irq_server->vka = vka;
    irq_server->thread_priority = priority;
    irq_server->server_threads = NULL;
    irq_server->simple = *simple;
    irq_server->affinity = affinity;

    /* If a fixed number of IRQs are requested, create and start the server threads */
    if (nirqs > -1) {
        struct irq_server_thread **server_thread;
        int n_nodes;
        int i;
        server_thread = &irq_server->server_threads;
        n_nodes = (nirqs + NIRQS_PER_NODE - 1) / NIRQS_PER_NODE;
        for (i = 0; i < n_nodes; i++) {
            *server_thread = irq_server_thread_new(vspace, vka, cspace, priority,
                                                   simple, label, sync_ep);
#if CONFIG_MAX_NUM_NODES > 1
            seL4_TCB_SetAffinity((*server_thread)->thread.tcb.cptr, affinity);
#endif
            server_thread = &(*server_thread)->next;
        }
    }

    *ret_irq_server = irq_server;
    return 0;
}

seL4_MessageInfo_t irq_server_wait_for_irq(irq_server_t irq_server, seL4_Word *badge_ret)
{
    seL4_MessageInfo_t msginfo;
    seL4_Word badge;

    /* Wait for an event */
    msginfo = api_recv(irq_server->delivery_ep, &badge, irq_server->reply.cptr);
    if (badge_ret) {
        *badge_ret = badge;
    }

    /* Forward to IRQ handlers */
    if (seL4_MessageInfo_get_label(msginfo) == irq_server->label) {
        irq_server_handle_irq_ipc(irq_server);
    }
    return msginfo;
}

/* end of interrupt server stuff */



/* A 32-bit badge contain for a VM contains two parts.
 * The high 16 bits are for the VM identity.
 * The low 16 bits are for identifiying a VCPU inside the VM
 */
#define VM_BADGE_SHIFT      (16)
#define VM_VCPU_BADGE(vm, vcpu)     (((vm) << VM_BADGE_SHIFT) | (vcpu))
#define VM_VCPU_BADGE_GET_VM(b)     ((b) >> VM_BADGE_SHIFT)
#define VM_VCPU_BADGE_GET_VCPU(b)   ((b) & 0xffff)
#define VM_BADGE            (1U << VM_BADGE_SHIFT)

#define VM_PRIO             100

#if CONFIG_MAX_NUM_NODES > 1
#define VM_LINUX_NAME       "linux-smp"
#else
#define VM_LINUX_NAME       "linux"
#endif

#if CONFIG_MAX_NUM_NODES == 2
#define VM_LINUX_DTB_NAME    "linux-smp2.dtb"
#elif CONFIG_MAX_NUM_NODES == 3
#define VM_LINUX_DTB_NAME    "linux-smp3.dtb"
#elif CONFIG_MAX_NUM_NODES == 4
#define VM_LINUX_DTB_NAME    "linux-smp4.dtb"
#else
#define VM_LINUX_DTB_NAME   "linux-dtb"
#endif

#define VM_NAME                 "Linux"

#define IRQSERVER_PRIO          (VM_PRIO + 1)
#define VMM_PRIO                (IRQSERVER_PRIO + 1)
#define IRQ_MESSAGE_LABEL       0xCAFE
#define IPI_MESSAGE_LABEL       0xBEEF
#define IRQINJ_MESSAGE_LABLE    0xDEEF

#define MAX_IRQ             20 //256

#define VM_CSPACE_SIZE_BITS 4
#define VM_CSPACE_SLOT      1
#define VM_FAULT_EP_SLOT    2

#ifndef DEBUG_BUILD
#define seL4_DebugHalt() do{ printf("Halting...\n"); while(1); } while(0)
#endif

typedef struct vm vm_t;

enum fault_width {
    WIDTH_DOUBLEWORD,
    WIDTH_WORD,
    WIDTH_HALFWORD,
    WIDTH_BYTE
};

struct fault {
    vm_t *vm;
    struct vcpu_info *vcpu;
    int    vcpu_id;
    /// Reply capability to the faulting TCB
    cspacepath_t reply_cap;
    seL4_UserContext regs;
    seL4_Word base_addr;
    seL4_Word addr;
    seL4_Word ip;
    seL4_Word data;
    seL4_Word fsr;
    bool is_prefetch;
    bool is_wfi;
    int stage;
    enum fault_width width;
    int content;
    uint32_t riscv_inst;
    riscv_inst_t decoded_inst; 
};

typedef struct fault fault_t;

void decode_inst(fault_t *f);

enum devid {
    DEV_RAM,
    DEV_UART0,
    DEV_PLIC,
    DEV_NDEVICES,
};

/**
 * Device description
 */
struct device {
/// Logical identifier for internal use
    enum devid devid;
/// A string representation of the device. Useful for debugging
    const char *name;
/// The physical address of the device */
    seL4_Word pstart;
/// Device mapping size */
    seL4_Word size;

/// Fault handler */
    int (*handle_page_fault)(struct device *d, vm_t *vm, fault_t *fault);
/// device emulation private data */
    void *priv;
};

#define MAX_DEVICES_PER_VM  10
#define MAX_NUM_VCPUS       4

#if CONFIG_MAX_NUM_NODES > MAX_NUM_VCPUS
#error CONFIG_MAX_NUM_NODES is greater than MAX_NUM_VCPUS
#endif

/* Support aarch64 only */
typedef struct vcpu_info {
    uint64_t        hart_id;     /* MPIDR */
    uint64_t        entry_point;    /* entry point in guest physical address */
    vka_object_t    tcb;
    vka_object_t    vcpu;
    fault_t         *fault;
    /* this is the VMM thread for this VCPU for handling VCPU faluts */
    sel4utils_thread_t vmm_thread;
    seL4_CPtr       fault_ep;
    int             affinity;
    irq_server_t    irq_server;
    int             suspended;
} vcpu_info_t;

struct vm {
    const char              *name;
    const char              *dtb_name;
    vka_t                   *vka;
    simple_t                *simple;
    vspace_t                *vmm_vspace;
    vspace_t                vm_vspace;
    seL4_Word               vm_badge;
    seL4_CPtr               vmm_endpoint;
    sel4utils_alloc_data_t  data;
    vka_object_t            cspace;
    vka_object_t            pd;
    vcpu_info_t             vcpus[MAX_NUM_VCPUS];
    int                     nvcpus;
    seL4_Word               csapce_root_data;
    void                    *entry_point;
    int                     ndevices;
    struct device           devices[MAX_DEVICES_PER_VM];
}; 

/* allocator static pool */
#define ALLOCATOR_STATIC_POOL_SIZE ((1 << seL4_PageBits) * 4000)
static char allocator_mem_pool[ALLOCATOR_STATIC_POOL_SIZE];

vka_t _vka;
simple_t _simple;
vspace_t _vspace;
sel4utils_alloc_data_t _alloc_data;
allocman_t *_allocator;
seL4_CPtr _fault_endpoint;
irq_server_t _irq_server;

struct ps_io_ops _io_ops;

extern char _cpio_archive[];

typedef struct guest_vspace {
    /* We abuse struct ordering and this member MUST be the first
     * thing in the struct */
    struct sel4utils_alloc_data vspace_data;
} guest_vspace_t;


static inline seL4_CPtr vm_get_tcb(vm_t *vm, int vcpu)
{
    assert(vm != 0);
    assert(vcpu >= 0 && vcpu < vm->nvcpus);
    return vm->vcpus[vcpu].tcb.cptr;
}

static inline seL4_CPtr vm_get_vcpu(vm_t *vm, int vcpu)
{
    assert(vm != 0);
    assert(vcpu >= 0 && vcpu < vm->nvcpus);
    return vm->vcpus[vcpu].vcpu.cptr;
}

static inline vspace_t *vm_get_vspace(vm_t *vm)
{
    assert(vm != 0);
    return &vm->vm_vspace;
}

static inline fault_t *vm_get_fault(vm_t *vm, int vcpu)
{
    assert(vm != 0);
    assert(vcpu >= 0 && vcpu < vm->nvcpus);
    return vm->vcpus[vcpu].fault;
}

static fault_t *fault_init(vm_t *vm, vcpu_info_t *vcpu)
{
    fault_t *fault;
    int err;
    fault = (fault_t *)malloc(sizeof(*fault));
    if (fault != NULL) {
        fault->vm = vm;
        fault->vcpu = vcpu;
        /* Reserve a slot for saving reply caps */
        err = vka_cspace_alloc_path(vm->vka, &fault->reply_cap);
        if (err) {
            free(fault);
            fault = NULL;
        }
    }
    return fault;
}



#ifdef DEBUG_FAULTS
#define DFAULT(...) printf(__VA_ARGS__)
#else
#define DFAULT(...) do{}while(0)
#endif

#define CONTENT_REGS               BIT(0)
#define CONTENT_DATA               BIT(1)
#define CONTENT_INST               BIT(2)
#define CONTENT_WIDTH              BIT(3)
#define CONTENT_STAGE              BIT(4)


#define RISCV_STORE_ACCESS_FAULT    7
#define RISCV_LOAD_ACCESS_FAULT     5

static int fault_handled(fault_t *f)
{
    return f->stage == 0;
}

static seL4_Word fault_get_address(fault_t *f)
{
    return f->addr;
}

static int fault_is_prefetch(fault_t *f)
{
    return f->is_prefetch;
}

static inline int fault_is_data(fault_t *f)
{
    return !fault_is_prefetch(f);
}

static inline int fault_is_write(fault_t *f)
{
    return (f->fsr == RISCV_STORE_ACCESS_FAULT);
}

static inline int fault_is_read(fault_t *f)
{
    return !fault_is_write(f);
}

static inline seL4_Word fault_get_addr_word(fault_t *f)
{
    return fault_get_address(f) & ~(0x3U);
}

static seL4_UserContext* fault_get_ctx(fault_t *f)
{
    if ((f->content & CONTENT_REGS) == 0) {
        int err;
        vcpu_info_t *vcpu = f->vcpu;

        err = seL4_TCB_ReadRegisters(vcpu->tcb.cptr, false, 0,
                                     sizeof(f->regs) / sizeof(f->regs.pc),
                                     &f->regs);
        assert(!err);
        f->content |= CONTENT_REGS;
    }
    return &f->regs;
}

static int new_user_fault(fault_t *fault)
{
    seL4_Word ip, addr;
    seL4_Word is_prefetch;
    int err;
    vm_t *vm;

    vm = fault->vm;
    assert(vm);

    assert(fault_handled(fault));

    /* First store message registers on the stack to free our message regs */
    addr = 0;
    ip = seL4_GetMR(seL4_UserException_FaultIP);
    fault->riscv_inst = seL4_GetMR(seL4_UserException_Code);
    DFAULT("%s: New user fault @ 0x%x from PC 0x%x\n", vm->name, addr, ip);
    /* Create the fault object */
    fault->is_wfi = 0;
    fault->is_prefetch = false;
    fault->ip = ip;
    fault->base_addr = fault->addr = addr;
    fault->data = 0;
    fault->width = -1;
    if (fault_is_data(fault)) {
        if (fault_is_read(fault)) {
            /* No need to load data */
            fault->content = CONTENT_DATA;
        } else {
            fault->content = 0;
        }
        fault->stage = -1;
    } else {
        /* No need to load width or data */
        fault->content = CONTENT_DATA | CONTENT_WIDTH;
    }

    /* Gather additional information */
    assert(fault->reply_cap.capPtr);
    err = vka_cnode_saveCaller(&fault->reply_cap);
    assert(!err);

    return err;
}

static int new_fault(fault_t *fault)
{
    seL4_Word ip, addr, fsr;
    seL4_Word is_prefetch;
    int err;
    vm_t *vm;

    vm = fault->vm;
    assert(vm);

    assert(fault_handled(fault));

    /* First store message registers on the stack to free our message regs */
    is_prefetch = seL4_GetMR(seL4_VMFault_PrefetchFault);
    addr = seL4_GetMR(seL4_VMFault_Addr),
    fsr = seL4_GetMR(seL4_VMFault_FSR);
    ip = seL4_GetMR(seL4_VMFault_IP);
    fault->riscv_inst = seL4_GetMR(seL4_VMFault_Instruction);
    DFAULT("%s: New fault @ 0x%x from PC 0x%x\n", vm->name, addr, ip);
    /* Create the fault object */
    fault->is_wfi = 0;
    fault->is_prefetch = is_prefetch;
    fault->ip = ip;
    fault->base_addr = fault->addr = addr;
    fault->fsr = fsr;
    fault->data = 0;
    fault->width = -1;
    if (fault_is_data(fault)) {
        if (fault_is_read(fault)) {
            /* No need to load data */
            fault->content = CONTENT_DATA;
        } else {
            fault->content = 0;
        }
        fault->stage = -1;
    } else {
        /* No need to load width or data */
        fault->content = CONTENT_DATA | CONTENT_WIDTH;
    }

    /* Gather additional information */
    assert(fault->reply_cap.capPtr);
    err = vka_cnode_saveCaller(&fault->reply_cap);
    assert(!err);

    return err;
}

static int abandon_fault(fault_t *fault)
{
    /* Nothing to do here */
    DFAULT("%s: Release fault @ 0x%x from PC 0x%x\n",
           fault->vm->name, fault->addr, fault->ip);
    return 0;
}

static int restart_fault(fault_t *fault)
{
    /* Send the reply */
    fault->stage = 0;
    seL4_MessageInfo_t reply;
    reply = seL4_MessageInfo_new(0, 0, 0, 0);
    DFAULT("%s: Restart fault @ 0x%x from PC 0x%x\n",
           fault->vm->name, fault->addr, fault->ip);
    seL4_Send(fault->reply_cap.capPtr, reply);
    /* Clean up */
    return abandon_fault(fault);
}

static int ignore_fault(fault_t *fault)
{
    seL4_UserContext *regs;
    int err;
    vcpu_info_t *vcpu = fault->vcpu;
    assert(vcpu);
    regs = fault_get_ctx(fault);
    /* Advance the PC */
    regs->pc += 4;
    /* Write back CPU registers */
    err = seL4_TCB_WriteRegisters(vcpu->tcb.cptr, false, 0,
                                  sizeof(*regs) / sizeof(regs->pc), regs);
    assert(!err);
    if (err) {
        abandon_fault(fault);
        return err;
    }
    /* Reply to thread */
    return restart_fault(fault);
}

static int vm_add_device(vm_t *vm, const struct device *d)
{
    assert(d != NULL);
    if (vm->ndevices < MAX_DEVICES_PER_VM) {
        vm->devices[vm->ndevices++] = *d;
        return 0;
    } else {
        return -1;
    }
}

static int guest_vspace_map(vspace_t *vspace, seL4_CPtr cap, void *vaddr,
                            seL4_CapRights_t rights,
                            int cacheable, size_t size_bits)
{
    int error;
    /* perfrom the guest mapping */
    error = sel4utils_map_page_pd(vspace, cap, vaddr, rights, cacheable, size_bits);
    if (error) {
        return error;
    }
    return 0;
}

static int vmm_get_guest_vspace(vspace_t *loader, vspace_t *new_vspace, vka_t *vka, seL4_CPtr pd)
{
    int error;
    guest_vspace_t *vspace = malloc(sizeof(*vspace));
    if (!vspace) {
        ZF_LOGE("Malloc failed");
        return -1;
    }
    error = sel4utils_get_vspace_with_map(loader, new_vspace, &vspace->vspace_data, vka, pd, NULL, NULL, guest_vspace_map);
    if (error) {
        ZF_LOGE("Failed to create guest vspace");
        return error;
    }
    new_vspace->unmap_pages = NULL;
    return 0;
}


#define IRQ_UART    10

#if CONFIG_MAX_NUM_NODES > 1
#define IRQ_VCPU0_VTIMER    132
#define IRQ_VCPU1_VTIMER    133
#define IRQ_VCPU2_VTIMER    134
#define IRQ_VCPU3_VTIMER    135

#else

#define IRQ_VTIMER          130
#endif

#define SIP_TIMER       BIT(5)
#define SIP_EXTERNAL    BIT(9)

static int vmm_init(void)
{
    vka_object_t fault_ep_obj;
    vka_t* vka;
    simple_t* simple;
    vspace_t* vspace;
    int err;

    vka = &_vka;
    vspace = &_vspace;
    simple = &_simple;
    fault_ep_obj.cptr = 0;

    seL4_BootInfo *info = platsupport_get_bootinfo();

    simple_default_init_bootinfo(simple, info);

    _allocator = bootstrap_use_current_simple(simple, ALLOCATOR_STATIC_POOL_SIZE,
                                              allocator_mem_pool);
    assert(_allocator);

    allocman_make_vka(vka, _allocator);

    err = sel4utils_bootstrap_vspace_with_bootinfo_leaky(vspace,
                                                         &_alloc_data,
                                                         seL4_CapInitThreadPD,
                                                         vka,
                                                         info);
    assert(!err);

    /* Setup debug port: printf() is only reliable after this */
    platsupport_serial_setup_simple(NULL, simple, vka);

    /* Allocate an endpoint for listening to events */
    err = vka_alloc_endpoint(vka, &fault_ep_obj);
    assert(!err);
    _fault_endpoint = fault_ep_obj.cptr;

    err  = irq_server_new(vspace, vka, simple_get_cnode(simple), IRQSERVER_PRIO,
            simple, fault_ep_obj.cptr,
            IRQ_MESSAGE_LABEL, MAX_IRQ, 0, &_irq_server);
    assert(!err);
    return 0;
}

static int vm_start(vm_t* vm)
{
    return seL4_TCB_Resume(vm_get_tcb(vm, 0));
}

static int vm_stop(vm_t* vm)
{
    for (int i = 0; i < vm->nvcpus; i++) {
        seL4_TCB_Suspend(vm_get_tcb(vm, i));
    }
    return 0;
}


static int vm_event(vm_t* vm, seL4_MessageInfo_t tag, seL4_Word badge);

static void vmm_handler_thread(vm_t *vm, seL4_Word vcpu_index)
{
    seL4_CPtr ep = vm->vcpus[vcpu_index].fault_ep;
    vcpu_info_t *vcpu_info = &vm->vcpus[vcpu_index];
    int err = 0;
    while (1) {
        seL4_MessageInfo_t tag;
        seL4_Word sender_badge;
        tag= seL4_Recv(ep, &sender_badge);
        //printf("label %lx badge %lx\n", seL4_MessageInfo_get_label(tag), sender_badge);
        seL4_CPtr vcpu = vm_get_vcpu(vm,  vcpu_index);
        if (sender_badge == 0) {
            seL4_Word label;
            label = seL4_MessageInfo_get_label(tag);
            switch (label) {
                case IPI_MESSAGE_LABEL: {
                    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                    assert(!res.error);
                    res.value |= BIT(1);
                    seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, res.value);
                    if (vcpu_info->suspended) {
                        restart_fault(vcpu_info->fault);
                        vcpu_info->suspended = 0;
                    }
                    break;
                }
#ifndef CONFIG_DIRECT_VCPU_IRQ
                case IRQINJ_MESSAGE_LABLE: {
                    int irq = seL4_GetMR(0);
                    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                    assert(!res.error);
                    seL4_Word sip = res.value;
                    switch (irq) {
#if CONFIG_MAX_NUM_NODES > 1
                        case IRQ_VCPU0_VTIMER:
                        case IRQ_VCPU1_VTIMER:
                        case IRQ_VCPU2_VTIMER:
                        case IRQ_VCPU3_VTIMER:
#else
                        case IRQ_VTIMER:
#endif
                            sip |= BIT(5);
                            break;
                        default:
                            printf("not timer ! %d\n", irq);
                            assert(0);
                            break;
                    }
                    int err = seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, sip);
                    if (vcpu_info->suspended) {
                        // resume the VCPU if it is suspended (WFI)
                        restart_fault(vcpu_info->fault);
                        vcpu_info->suspended = 0;
                    }
                    assert(!err);
                    break;
                }
#endif
                case IRQ_MESSAGE_LABEL: {
                    irq_server_handle_irq_ipc(vcpu_info->irq_server);
                    break;
                }
                default:
                    printf("vcpu %d for IPC badge %d\n", (int)vcpu_index, sender_badge);
                    break;
            }
        } else {
            int vm_badge = VM_VCPU_BADGE_GET_VM(sender_badge);
            int vcpu = VM_VCPU_BADGE_GET_VCPU(sender_badge);
            if (vcpu != vcpu_index) {
                /* sanity check */
                printf("VCPU %d expected %d\n", vcpu, vcpu_index);
                assert(0);
            }
            if (vm_badge == VM_BADGE) {
                err = vm_event(vm, tag, sender_badge | vcpu_index);
                if (err) {
                    /* Shutdown */
                    vm_stop(vm);
                    printf("vm 0 halts\n");
                    seL4_DebugHalt();
                }
            }
        }
    }
}

static void irq_handler(struct irq_data *irq_data);

static struct irq_data *linux_irq_data[256] = { 0 };

struct virq_handle {
    int virq;
    void (*ack)(void *token);
    void *token;
    vm_t *vm;
    struct virq_handle *next;
};

typedef struct virq_handle *virq_handle_t;

static virq_handle_t vm_virq_new(vm_t* vm, int virq, void (*ack)(void*), void* token)
{
    struct virq_handle* virq_data;
    int err;

    virq_data = malloc(sizeof(*virq_data));
    if (!virq_data) {
        return NULL;
    }
    virq_data->virq = virq;
    virq_data->token = token;
    virq_data->ack = ack;
    virq_data->vm = vm;
    virq_data->next = NULL;
    return virq_data;
}

void do_irq_server_ack(void *token);
static int create_vmm_thread(vm_t *vm, vcpu_info_t *vcpu)
{
    int err = 0;

    simple_t *simple = vm->simple;
    seL4_CPtr cspace = simple_get_cnode(vm->simple);
    vspace_t *vspace = vm->vmm_vspace;
    sel4utils_thread_t *thread = &vcpu->vmm_thread;
    sel4utils_thread_config_t config = thread_config_default(simple, cspace, seL4_NilData, 0, VMM_PRIO+ 10);
    vka_object_t fault_ep;
    err = sel4utils_configure_thread_config(vm->vka, vspace, vspace, config, thread);
#if CONFIG_MAX_NUM_NODES > 1
    printf("set VMM %lx to %d\n", thread->tcb.cptr, vm->nvcpus);
    err = seL4_TCB_SetAffinity(thread->tcb.cptr, vm->nvcpus);
    seL4_DebugNameThread(thread->tcb.cptr, "vmm_1");
    assert(!err);
#endif
    err = vka_alloc_endpoint(vm->vka, &fault_ep);
    assert(!err);
    vcpu->fault_ep = fault_ep.cptr;
    err = sel4utils_start_thread(thread, (void *)vmm_handler_thread, vm, (void *)(seL4_Word)vm->nvcpus, 1);
    assert(!err);
    printf("Create VMM thread for VCPU %d\n", (int)vm->nvcpus);

    err = irq_server_new(vspace, vm->vka, cspace, IRQSERVER_PRIO,
            simple, fault_ep.cptr, IRQ_MESSAGE_LABEL, 1, vm->nvcpus,
            &vcpu->irq_server);
    assert(!err);
    printf("Create IRQ server for for VCPU %d\n", vm->nvcpus);
    void (* handler)(struct irq_data *);
    handler = &irq_handler;
    struct irq_data *irq_data;
    virq_handle_t virq;
    switch (vm->nvcpus) {
        case 1:
            irq_data = irq_server_register_irq(vcpu->irq_server, 133, handler, NULL);
            assert(irq_data);
            linux_irq_data[133] = irq_data;

            virq = vm_virq_new(vm, 133, &do_irq_server_ack, irq_data);
            virq->next = irq_data->token;
            irq_data->token = (void *)virq;
            break;
        case 2:
            irq_data = irq_server_register_irq(vcpu->irq_server, 134, handler, NULL);
            assert(irq_data);
            linux_irq_data[134] = irq_data;

            virq = vm_virq_new(vm, 134, &do_irq_server_ack, irq_data);
            virq->next = irq_data->token;
            irq_data->token = (void *)virq;
            break;
        case 3:
            irq_data = irq_server_register_irq(vcpu->irq_server, 135, handler, NULL);
            assert(irq_data);
            linux_irq_data[135] = irq_data;
            virq = vm_virq_new(vm, 135, &do_irq_server_ack, irq_data);
            virq->next = irq_data->token;
            irq_data->token = (void *)virq;
            break;
        default:
            printf("unsupported %d\n", vm->nvcpus);
            assert(0);
    }

    return 0;
}

static int vm_add_vcpu(vm_t *vm, uint64_t entry_point, uint64_t hart_id, seL4_Word dtb)
{
    if (vm->nvcpus == MAX_NUM_VCPUS) {
        printf("Exceed max supported VCPUs %d\n", MAX_NUM_VCPUS);
        return -1;
    }
    seL4_Word null_cap_data = seL4_NilData;
    seL4_Word vm_vcpu_badge = VM_VCPU_BADGE(vm->vm_badge, (vm->nvcpus));


    seL4_Word cspace_root_data = api_make_guard_skip_word(seL4_WordBits - VM_CSPACE_SIZE_BITS);
    cspacepath_t src, dst;
    int err;

#ifdef CONFIG_PER_VCPU_VMM
    create_vmm_thread(vm, &vm->vcpus[vm->nvcpus]);
    /* per thread VMM */
    seL4_CPtr fault_ep = vm->vcpus[vm->nvcpus].fault_ep;
#else
    /* one VMM */
    seL4_CPtr fault_ep = vm->vmm_endpoint;
#endif

    /* Badge the endpoint */
    vka_cspace_make_path(vm->vka, fault_ep, &src);
    err = vka_cspace_alloc_path(vm->vka, &dst);
    assert(!err);
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, vm_vcpu_badge);
    assert(!err);

    /* Copy it to the cspace of the VM for fault IPC */
    src = dst;
    dst.root = vm->cspace.cptr;
    dst.capPtr = VM_FAULT_EP_SLOT + (vm->nvcpus);
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_copy(&dst, &src, seL4_AllRights);
    assert(!err);

    err = vka_alloc_tcb(vm->vka, &vm->vcpus[vm->nvcpus].tcb);
    assert(!err);
    err = seL4_TCB_Configure(vm->vcpus[vm->nvcpus].tcb.cptr, (VM_FAULT_EP_SLOT + vm->nvcpus),
                             vm->cspace.cptr, cspace_root_data,
                             vm->pd.cptr, null_cap_data, 0, seL4_CapNull);
    assert(!err);
    err = seL4_TCB_SetSchedParams(vm->vcpus[vm->nvcpus].tcb.cptr, simple_get_tcb(vm->simple), VM_PRIO, VM_PRIO);
    assert(!err);

    /* Create VCPU */
    err = vka_alloc_vcpu(vm->vka, &vm->vcpus[vm->nvcpus].vcpu);
    assert(!err);
    err = seL4_RISCV_VCPU_SetTCB(vm->vcpus[vm->nvcpus].vcpu.cptr, vm->vcpus[vm->nvcpus].tcb.cptr);
    assert(!err);
    vm->vcpus[vm->nvcpus].hart_id = hart_id;
    vm->vcpus[vm->nvcpus].entry_point = entry_point;
    vm->vcpus[vm->nvcpus].fault = fault_init(vm, &vm->vcpus[vm->nvcpus]);
    vm->vcpus[vm->nvcpus].suspended = 0;
    assert(vm->vcpus[vm->nvcpus].fault);

    seL4_UserContext regs;
    bzero(&regs, sizeof(regs));
    regs.a0 = hart_id;
    regs.a1 = dtb;
    regs.pc = entry_point;
    err = seL4_TCB_WriteRegisters(vm->vcpus[vm->nvcpus].tcb.cptr, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);

#if CONFIG_MAX_NUM_NODES > 1
    err = seL4_TCB_SetAffinity(vm->vcpus[vm->nvcpus].tcb.cptr, vm->nvcpus);
    vm->vcpus[vm->nvcpus].affinity = vm->nvcpus;
    assert(!err);
#endif
    {
        char buf[64];
        snprintf(buf, 64, "vcpu %d", (int)hart_id);
        seL4_DebugNameThread(vm->vcpus[vm->nvcpus].tcb.cptr, buf);
    }
    printf("Resume VCPU %lx %d\n", hart_id, vm->nvcpus);
    err = seL4_TCB_Resume(vm->vcpus[vm->nvcpus].tcb.cptr);
    assert(!err);

    vm->nvcpus++;
    return 0;
}

static int vm_create(const char* name, int priority,
          seL4_CPtr vmm_endpoint, seL4_Word vm_badge,
          vka_t *vka, simple_t *simple, vspace_t *vmm_vspace,
          ps_io_ops_t *io_ops,
          vm_t *vm)
{

    seL4_Word null_cap_data = seL4_NilData;
    seL4_Word cspace_root_data;
    cspacepath_t src, dst;

    int err;
    bzero(vm, sizeof(vm_t));
    vm->name = name;
    vm->entry_point = NULL;
    vm->vka = vka;
    vm->simple = simple;
    vm->vmm_vspace = vmm_vspace;
    vm->ndevices = 0;
    vm->vmm_endpoint = vmm_endpoint;
    vm->nvcpus = 1;
    vm->vm_badge = VM_BADGE;
    int vcpu_id = 0;

    /* Create a cspace */
    err = vka_alloc_cnode_object(vka, VM_CSPACE_SIZE_BITS, &vm->cspace);
    assert(!err);
    vka_cspace_make_path(vka, vm->cspace.cptr, &src);

    cspace_root_data = api_make_guard_skip_word(seL4_WordBits - VM_CSPACE_SIZE_BITS);

    dst.root = vm->cspace.cptr;
    dst.capPtr = VM_CSPACE_SLOT;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, cspace_root_data);
    assert(!err);

    /* Create a vspace */
    err = vka_alloc_vspace_root(vka, &vm->pd);
    assert(!err);
    err = simple_ASIDPool_assign(simple, vm->pd.cptr);
    assert(err == seL4_NoError);
    err = vmm_get_guest_vspace(vmm_vspace, &vm->vm_vspace, vka, vm->pd.cptr);
    assert(!err);

    /* Badge the endpoint */
    vka_cspace_make_path(vka, vmm_endpoint, &src);
    err = vka_cspace_alloc_path(vka, &dst);
    assert(!err);
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, vm_badge);
    assert(!err);
    /* Copy it to the cspace of the VM for fault IPC */
    src = dst;
    dst.root = vm->cspace.cptr;
    dst.capPtr = VM_FAULT_EP_SLOT;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_copy(&dst, &src, seL4_AllRights);
    assert(!err);

    /* Create TCB */
    err = vka_alloc_tcb(vka, &vm->vcpus[vcpu_id].tcb);
    assert(!err);
    err = seL4_TCB_Configure(vm_get_tcb(vm, vcpu_id), VM_FAULT_EP_SLOT,
                             vm->cspace.cptr, cspace_root_data,
                             vm->pd.cptr, null_cap_data, 0, seL4_CapNull);
    assert(!err);

    err = seL4_TCB_SetSchedParams(vm_get_tcb(vm, vcpu_id), simple_get_tcb(simple), priority, priority);
    assert(!err);

    seL4_DebugNameThread(vm_get_tcb(vm, vcpu_id), "vmlinux");

    /* Create VCPU */
    err = vka_alloc_vcpu(vka, &vm->vcpus[vcpu_id].vcpu);
    assert(!err);
    err = seL4_RISCV_VCPU_SetTCB(vm->vcpus[vcpu_id].vcpu.cptr, vm_get_tcb(vm, vcpu_id));
    assert(!err);

    /* Initialise fault system */
    vm->vcpus[vcpu_id].fault = fault_init(vm, &vm->vcpus[vcpu_id]);
    vm->vcpus[vcpu_id].affinity = 0;
    vm->vcpus[vcpu_id].fault_ep = vmm_endpoint;
    vm->vcpus[vcpu_id].suspended = 0;
    return err;
}


static int vm_set_bootargs(vm_t *vm, seL4_Word pc, seL4_Word hartid , seL4_Word dtb)
{
    seL4_UserContext regs;
    seL4_CPtr tcb;
    int err;
    assert(vm);
    /* Write CPU registers */
    tcb = vm_get_tcb(vm, 0);
    err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    regs.pc = pc;
    regs.a0 = hartid;
    regs.a1 = dtb;
    err = seL4_TCB_WriteRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    return err;
}

/* dummy global for libsel4muslcsys */
extern char _cpio_archive[1];
extern char _cpio_archive_end[1];

#define MAP_PAGE_BITS   21
#define MAP_PAGE_SIZE   (1 << MAP_PAGE_BITS)
#define MAP_PAGE_MASK   (MAP_PAGE_SIZE - 1)

static void *map_ram(vspace_t *vspace, vspace_t *vmm_vspace, vka_t *vka, uintptr_t vaddr)
{
    vka_object_t frame_obj;
    cspacepath_t frame[2];

    reservation_t res;
    void *addr;
    int err;

    addr = (void*)(vaddr & ~MAP_PAGE_MASK);

    /* reserve vspace */
    res = vspace_reserve_range_at(vspace, addr, MAP_PAGE_SIZE, seL4_AllRights, 1);
    if (!res.res) {
        ZF_LOGF("Failed to reserve range");
        return NULL;
    }

    /* Create a frame */
    err = vka_alloc_frame_maybe_device(vka, MAP_PAGE_BITS, true, &frame_obj);
    if (err) {
        ZF_LOGF("Failed vka_alloc_frame_maybe_device");
        vspace_free_reservation(vspace, res);
        return NULL;
    }

    vka_cspace_make_path(vka, frame_obj.cptr, &frame[0]);

    err = vka_cspace_alloc_path(vka, &frame[1]);
    if (err) {
        ZF_LOGF("Failed vka_cspace_alloc_path");
        vka_free_object(vka, &frame_obj);
        vspace_free_reservation(vspace, res);
        return NULL;
    }

    err = vka_cnode_copy(&frame[1], &frame[0], seL4_AllRights);
    if (err) {
        ZF_LOGF("Failed vka_cnode_copy");
        vka_cspace_free(vka, frame[1].capPtr);
        vka_free_object(vka, &frame_obj);
        vspace_free_reservation(vspace, res);
        return NULL;
    }


    /* Map in the frame */
    err = vspace_map_pages_at_vaddr(vspace, &frame[0].capPtr, NULL, addr, 1, MAP_PAGE_BITS, res);
    vspace_free_reservation(vspace, res);
    if (err) {
        ZF_LOGF("Failed vspace_map_pages_at_vaddr");
        vka_cspace_free(vka, frame[1].capPtr);
        vka_free_object(vka, &frame_obj);
        return NULL;
    }

    /* Map into the vspace of the VMM to zero memory */
    void *vmm_addr;
    seL4_CapRights_t rights = seL4_AllRights;
    seL4_CPtr cap = frame[1].capPtr;
    vmm_addr = vspace_map_pages(vmm_vspace, &cap, NULL, rights, 1, MAP_PAGE_BITS, true);
    if (vmm_addr == NULL) {
        ZF_LOGF("Failed vspace_map_pages");
        vspace_unmap_pages(vspace, (void*)addr, 1, MAP_PAGE_BITS, vka);
        vka_cspace_free(vka, frame[1].capPtr);
        vka_free_object(vka, &frame_obj);
        return NULL;
    }
    memset(vmm_addr, 0, MAP_PAGE_SIZE);
    /* This also frees the cspace slot we made.  */
    vspace_unmap_pages(vmm_vspace, (void*)vmm_addr, 1, MAP_PAGE_BITS, vka);

    return addr;
}

static void *map_vm_ram(vm_t *vm, uintptr_t vaddr)
{
    return map_ram(vm_get_vspace(vm), vm->vmm_vspace, vm->vka, vaddr);
}

static int vm_install_ram_only_device(vm_t *vm, const struct device *device) {
    struct device d;
    uintptr_t paddr;
    int err;
    d = *device;
    for (paddr = d.pstart; paddr - d.pstart < d.size; paddr += MAP_PAGE_SIZE) {
        void *addr;
        addr = map_vm_ram(vm, paddr);
        if (!addr) {
            return -1;
        }
    }
    err = vm_add_device(vm, &d);
    assert(!err);
    return err;
}


#define DEBUG_MAPPINGS

#ifdef DEBUG_MAPPINGS
#define DMAP(...) printf(__VA_ARGS__)
#else
#define DMAP(...) do{}while(0)
#endif


static void *map_device(vspace_t *vspace, vka_t *vka, simple_t *simple, uintptr_t paddr,
           uintptr_t _vaddr, seL4_CapRights_t rights)
{
    cspacepath_t frame;
    void *vaddr;
    int err;
    int cache = 0;

    paddr &= ~0xfff;
    vaddr = (void *)(_vaddr &= ~0xfff);

    if (paddr < 0x80000000) cache = 0;

    /* Alocate a slot */
    err = vka_cspace_alloc_path(vka, &frame);
    assert(!err);
    if (err) {
        printf("Failed to allocate cslot\n");
        return NULL;
    }

    /* Find the device cap */
    seL4_Word cookie;
    err = vka_utspace_alloc_at(vka, &frame, kobject_get_type(KOBJECT_FRAME, 12), 12, paddr, &cookie);
    if (err) {
        err = simple_get_frame_cap(simple, (void *)paddr, 12, &frame);
        if (err) {
            printf("Failed to find device cap for 0x%x\n", (uint32_t)paddr);
            vka_cspace_free(vka, frame.capPtr);
            return NULL;
        }
    }
    /* Map the device */
    if (vaddr) {
        reservation_t res;
        res = vspace_reserve_range_at(vspace, vaddr, 0x1000, rights, cache);
        assert(res.res);
        if (!res.res) {
            printf("Failed to reserve vspace\n");
            vka_cspace_free(vka, frame.capPtr);
            return NULL;
        }
        /* Map in the page */
        err = vspace_map_pages_at_vaddr(vspace, &frame.capPtr, NULL, vaddr,
                                        1, 12, res);
        vspace_free_reservation(vspace, res);
    } else {
        vaddr = vspace_map_pages(vspace, &frame.capPtr, NULL, rights, 1, 12, cache);
        err = (vaddr == 0);
    }
    assert(!err);
    if (err) {
        printf("Failed to provide memory\n");
        vka_cspace_free(vka, frame.capPtr);
        return NULL;
    }
    DMAP("Mapped device ipa0x%lx->p0x%lx\n", vaddr, paddr);
    return vaddr;
}

void *map_vm_device(vm_t *vm, uintptr_t pa, uintptr_t va, seL4_CapRights_t rights)
{
    lock(&vm_map_device_lock);
    void *ret = map_device(vm_get_vspace(vm), vm->vka, vm->simple, pa, va, rights);
    unlock(&vm_map_device_lock);
    return ret;
}

static void *map_emulated_device_pages(vm_t *vm, struct device *d)
{
    cspacepath_t vm_frame, vmm_frame;
    vspace_t *vm_vspace, *vmm_vspace;
    void *vm_addr, *vmm_addr, *ret;;
    reservation_t vm_res, vmm_res;
    vka_object_t frame;
    vka_t *vka;
    size_t size, remain_size;
    int err;

    vka = vm->vka;
    vm_addr = (void *)d->pstart;
    size = d->size;
    remain_size = size;
    vm_vspace = vm_get_vspace(vm);
    vmm_vspace = vm->vmm_vspace;
    assert(size % 0x1000 == 0);

    /* make reservations first */
    vm_res = vspace_reserve_range_at(vm_vspace, vm_addr, size, seL4_NoRights, 0); // can read
    assert(vm_res.res);
    if (!vm_res.res) {
        return NULL;
    }

    vmm_res = vspace_reserve_range_aligned(vmm_vspace, size, 12, seL4_AllRights, 1, &vmm_addr);
    assert(vmm_res.res);
    if (!vmm_res.res) {
        vspace_free_reservation(vmm_vspace, vm_res);
        return NULL;
    }
    ret = vmm_addr;
    DMAP("Mapping emulated device ipa0x%p size 0x%lx\n", vm_addr, size);
    for (; remain_size > 0; remain_size -= 0x1000) {
        /* Create a frame (and a copy for the VMM) */
        err = vka_alloc_frame(vka, 12, &frame);
        assert(!err);
        if (err) {
            return NULL;
        }
        vka_cspace_make_path(vka, frame.cptr, &vm_frame);
        err = vka_cspace_alloc_path(vka, &vmm_frame);
        assert(!err);
        if (err) {
            vka_free_object(vka, &frame);
            return NULL;
        }
        err = vka_cnode_copy(&vmm_frame, &vm_frame, seL4_AllRights);
        assert(!err);
        if (err) {
            vka_cspace_free(vka, vm_frame.capPtr);
            vka_free_object(vka, &frame);
            return NULL;
        }

        /* Map the frame to the VM */

        err = vspace_map_pages_at_vaddr(vm_vspace, &vm_frame.capPtr, NULL, vm_addr,
                                        1, 12, vm_res);
        //vspace_free_reservation(vm_vspace, vm_res);
        assert(!err);
        if (err) {
            printf("Failed to provide memory\n");
            vka_cspace_free(vka, vm_frame.capPtr);
            vka_cspace_free(vka, vmm_frame.capPtr);
            vka_free_object(vka, &frame);
            return NULL;
        }
        err = vspace_map_pages_at_vaddr(vmm_vspace, &vmm_frame.capPtr, NULL, vmm_addr,
                                    1, 12, vmm_res);
        assert(!err);
        if (err) {
            return NULL;
        }
        vm_addr += 0x1000;
        vmm_addr += 0x1000;
    }

    return ret;
}

static int handle_ram_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    /* TODO */
    return 0;
}

#define RAM_BASE  0x80000000
#define RAM_SIZE  0x08000000

struct device ram_dev = {
    .devid              = DEV_RAM,
    .name               = "ram",
    .pstart             = RAM_BASE,
    .size               = RAM_SIZE,
    .handle_page_fault  = handle_ram_fault,
    .priv               = NULL,
};

#define PLIC_BASE  0xc000000
#define PLIC_SIZE  0x4000000

#define PLIC_INT_SOURCE_START   1
#define PLIC_INT_SOURCE_END     136

/* offsets of registers */
#define PLIC_PRIORITY_START     0x4
#define PLIC_PRIORITY_END       0x224
#define PLIC_PENDING_START      0x1000
#define PLIC_PENDING_END        0x1014
#define PLIC_H0_ENABLE_START    0x2000
#define PLIC_H0_EANBLE_END      0x2010
#define PLIC_H1_ENABLE_START    0x2010
#define PLIC_H1_ENABLE_END      0x2020
#define PLIC_H2_ENABLE_START    0x2020
#define PLIC_H2_ENABLE_END      0x2030
#define PLIC_H3_ENABLE_START    0x2030
#define PLIC_H3_ENABLE_END      0x2040
#define PLIC_H4_ENABLE_STAR     0x2040
#define PLIC_H4_ENABLE_END      0x2050
#define PLIC_H0_THRESHOLD_START 0x200000
#define PLIC_H0_THRESHOLD_END   0x200004
#define PLIC_H0_CC_START        0x200004
#define PLIC_H0_CC_END          0x200008
#define PLIC_H1_THRESHOLD_START 0x200008
#define PLIC_H1_THRESHOLD_END   0x20000c
#define PLIC_H1_CC_START        0x20000c
#define PLIC_H1_CC_END          0x200010
#define PLIC_H2_THRESHOLD_START 0x200010
#define PLIC_H2_THRESHOLD_END   0x200014
#define PLIC_H2_CC_START        0x200014
#define PLIC_H2_CC_END          0x200018
#define PLIC_H3_THRESHOLD_START 0x200018
#define PLIC_H3_THRESHOLD_END   0x20001c
#define PLIC_H3_CC_START        0x20001c
#define PLIC_H3_CC_END          0x200020
#define PLIC_H4_THRESHOLD_START 0x200020
#define PLIC_H4_THRESHOLD_END   0x200024
#define PLIC_H4_CC_START        0x200024
#define PLIC_H4_CC_END          0x200028

#define IRQS_PER_WORD           32


static inline seL4_Word get_reg(seL4_UserContext *regs, int index)
{

    switch (index) {
        // X0 is always ZERO
        case 0:
            return 0;
        case 1:
            return regs->ra;
        case 2:
            return regs->sp;
        case 3:
            return regs->gp;
        case 4:
            return regs->tp;
        case 5:
            return regs->t0;
        case 6:
            return regs->t1;
        case 7:
            return regs->t2;
        case 8:
            return regs->s0;
        case 9:
            return regs->s1;
        case 10:
            return regs->a0;
        case 11:
            return regs->a1;
        case 12:
            return regs->a2;
        case 13:
            return regs->a3;
        case 14:
            return regs->a4;
        case 15:
            return regs->a5;
        case 16:
            return regs->a6;
        case 17:
            return regs->a7;
        case 18:
            return regs->s2;
        case 19:
            return regs->s3;
        case 20:
            return regs->s4;
        case 21:
            return regs->s5;
        case 22:
            return regs->s6;
        case 23:
            return regs->s7;
        case 24:
            return regs->s8;
        case 25:
            return regs->s9;
        case 26:
            return regs->s10;
        case 27:
            return regs->s11;
        case 28:
            return regs->t3;
        case 29:
            return regs->t4;
        case 30:
            return regs->t5;
        case 31:
            return regs->t6;
        default:
            printf("Invalid index %d\n", index);
            assert(0);
    }
}


static inline void set_reg(seL4_UserContext *regs, int index, seL4_Word v)
{
    switch (index) {
        // X0 is always ZERO
        case 0:
            return;
        case 1:
            regs->ra = v;
            return;
        case 2:
            regs->sp = v;
            return;
        case 3:
            regs->gp = v;
            return;
        case 4:
            regs->tp = v;
            return;
        case 5:
            regs->t0 = v;
            return;
        case 6:
            regs->t1 = v;
            return;
        case 7:
            regs->t2 = v;
            return;
        case 8:
            regs->s0 = v;
            return;
        case 9:
            regs->s1 = v;
            return;
        case 10:
            regs->a0 = v;
            return;
        case 11:
            regs->a1 = v;
            return;
        case 12:
            regs->a2 = v;
            return;
        case 13:
            regs->a3 = v;
            return;
        case 14:
            regs->a4 = v;
            return;
        case 15:
            regs->a5 = v;
            return;
        case 16:
            regs->a6 = v;
            return;
        case 17:
            regs->a7 = v;
            return;
        case 18:
            regs->s2 = v;
            return;
        case 19:
            regs->s3 = v;
            return;
        case 20:
            regs->s4 = v;
            return;
        case 21:
            regs->s5 = v;
            return;
        case 22:
            regs->s6 = v;
            return;
        case 23:
            regs->s7 = v;
            return;
        case 24:
            regs->s8 = v;
            return;
        case 25:
            regs->s9 = v;
            return;
        case 26:
            regs->s10 = v;
            return;
        case 27:
            regs->s11 = v;
            return;
        case 28:
            regs->t3 = v;
            return;
        case 29:
            regs->t4 = v;
            return;
        case 30:
            regs->t5 = v;
            return;
        case 31:
            regs->t6 = v;
            return;
        default:
            printf("Invalid index %d\n", index);
            assert(0);
    }
}

static int plic_pending_irq = 0;
static void *plic_pending_irq_token = NULL;

void do_irq_server_ack(void *token);

static int handle_plic_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    uintptr_t addr = fault->addr;
    uintptr_t offset = addr - PLIC_BASE;
    int write = fault_is_write(fault);
    vcpu_info_t *vcpu_info = fault->vcpu;
    assert(vcpu_info);
    void *vmm_va = (void *)d->priv;
    decode_inst(fault);
    seL4_UserContext *regs = fault_get_ctx(fault); 
    riscv_inst_t *ri = &fault->decoded_inst;
    switch (ri->opcode) {
        case ST_OP: {
            // 32bit!!!
            assert(fault->width == WIDTH_WORD);
            uint32_t data = (uint32_t)get_reg(regs, ri->rs2);
            uint32_t *addr = (uint32_t *)(vmm_va + offset);
            *addr = data;
           // printf("pc %lx store data %x to %p %lx\n", regs->pc, data, addr, offset);
            if ((offset >= PLIC_H0_CC_START && offset < PLIC_H0_CC_END) ||
                (offset >= PLIC_H1_CC_START && offset < PLIC_H1_CC_END) ||
                (offset >= PLIC_H2_CC_START && offset < PLIC_H2_CC_END)) {
                //printf("cc %d\n", vcpu_info->hart_id);
                plic_pending_irq = 0;
                do_irq_server_ack(plic_pending_irq_token);
            }
            ignore_fault(fault);
            return 0;
        }
        case LD_OP: {
            assert(fault->width == WIDTH_WORD);
            uint32_t data = 0;
            if ((offset >= PLIC_H0_CC_START && offset < PLIC_H0_CC_END) ||
                (offset >= PLIC_H1_CC_START && offset < PLIC_H1_CC_END) ||
                (offset >= PLIC_H2_CC_START && offset < PLIC_H2_CC_END)) {
                data = plic_pending_irq;
                //printf("c %d %d\n", data, vcpu_info->hart_id);
                plic_pending_irq = 0;
            } else {
                data = *(uint32_t *)(vmm_va + offset);
            }
            seL4_Word reg = get_reg(regs, ri->rd);
            reg &= 0xffffffff00000000;
            reg |= data;
            set_reg(regs, ri->rd, reg);
            ignore_fault(fault);
            return 0;
        }

        default:
            printf("Unhandled %d\n", ri->opcode);
            break;
    }
    return 0;
}

struct device plic_dev = {
    .devid              = DEV_PLIC,
    .name               = "plic",
    .pstart             = PLIC_BASE,
    .size               = PLIC_SIZE,
    .handle_page_fault  = handle_plic_fault,
    .priv               = NULL,
};


static int plic_set_pending(int irq, void *token)
{
    uintptr_t offset = irq / IRQS_PER_WORD;
    int bit = irq % IRQS_PER_WORD;

    plic_pending_irq = irq;
    plic_pending_irq_token = token;
    assert(plic_dev.priv != NULL);
    uint32_t *pending = (uint32_t *)(plic_dev.priv + PLIC_PENDING_START);
    pending += offset;
    *pending |= BIT(bit);
    return 0;
}

static int vm_install_plic(vm_t *vm)
{
    void *vaddr = map_emulated_device_pages(vm, &plic_dev);
    if (vaddr == NULL) return -1;

    plic_dev.priv = vaddr;
    bzero(vaddr, PLIC_SIZE);
    return vm_add_device(vm, &plic_dev); 
}

static int linux_irq[] = {
    IRQ_UART,       // 8250
#if CONFIG_MAX_NUM_NODES > 1
    IRQ_VCPU0_VTIMER,
#if CONFIG_MAX_NUM_NODES >= 2
    IRQ_VCPU1_VTIMER,
#endif
#if CONFIG_MAX_NUM_NODES >= 3
    IRQ_VCPU2_VTIMER,
#endif
#if CONFIG_MAX_NUM_NODES >= 4
    IRQ_VCPU3_VTIMER,
#endif
#else
    IRQ_VTIMER      // vtimer
#endif
};

static inline vcpu_info_t *get_vcpu_by_irq(vm_t *vm, int irq)
{
    switch (irq) {
#if CONFIG_MAX_NUM_NODES > 1
        case IRQ_VCPU0_VTIMER:
            return &vm->vcpus[0];
        case IRQ_VCPU1_VTIMER:
            return &vm->vcpus[1];
        case IRQ_VCPU2_VTIMER:
            return &vm->vcpus[2];
        case IRQ_VCPU3_VTIMER:
            return &vm->vcpus[3];
#endif
        default:
            return &vm->vcpus[0];
    }
}

void do_irq_server_ack(void *token)
{
    assert(token != NULL);
    vm_t *vm = (vm_t *)_irq_server->vm;
    struct irq_data* irq_data = (struct irq_data*)token;

    vcpu_info_t *vcpu_info = get_vcpu_by_irq(vm, irq_data->irq);
    seL4_CPtr vcpu = vcpu_info->vcpu.cptr;

#if CONFIG_MAX_NUM_NODES > 1
#ifdef CONFIG_PER_VCPU_VMM
    assert(vcpu_info->affinity == 0);
#else
    seL4_TCB_SetAffinity(seL4_CapInitThreadTCB, vcpu_info->affinity);
#endif
#endif

    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
    assert(!res.error);

    /* clear the externall pending */
    seL4_Word sip = res.value;
    switch (irq_data->irq) {
        case 130:
        case 132:
        case 133:
        case 134:
        case 135:
            assert(0);
            sip &= ~BIT(5);
            break;
        default:
            sip &= ~SIP_EXTERNAL;
            break;
    }
    int err = seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, sip);
    assert(!err);
    irq_data_ack_irq(irq_data);
}

static int vm_inject_timer_interrupt(vm_t *vm, fault_t *fault)
{
    assert(fault);
    seL4_CPtr vcpu = fault->vcpu->vcpu.cptr;
    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
    assert(!res.error);
    seL4_Word sip = res.value;
    res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIE);
    assert(!res.error);
    seL4_Word sie = res.value;
    sip |= SIP_TIMER;
    int err = seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, sip);
    assert(!err);
    return err;
}

static int vm_inject_IRQ(virq_handle_t virq)
{
    vm_t *vm = virq->vm;

    vcpu_info_t *vcpu_info = get_vcpu_by_irq(vm, virq->virq);
    seL4_CPtr vcpu = vcpu_info->vcpu.cptr;

#if CONFIG_MAX_NUM_NODES > 1
#ifndef CONFIG_PER_VCPU_VMM
    seL4_TCB_SetAffinity(seL4_CapInitThreadTCB, vcpu_info->affinity);
#endif
#endif

    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
    assert(!res.error);
    seL4_Word sip = res.value;

    switch (virq->virq) {
#if CONFIG_MAX_NUM_NODES > 1
        case IRQ_VCPU0_VTIMER:
        case IRQ_VCPU1_VTIMER:
        case IRQ_VCPU2_VTIMER:
        case IRQ_VCPU3_VTIMER:
#else
        case IRQ_VTIMER:
#endif
            sip |= SIP_TIMER;
            break;
        default:
            /* set the external pending */
            plic_set_pending(virq->virq, virq->token);
            sip |= SIP_EXTERNAL;
            break;
    }
    int err = seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, sip);
    assert(!err);

    if (vcpu_info->suspended) {
        restart_fault(vcpu_info->fault);
        vcpu_info->suspended = 0;
    }
    return 0;
}

static void irq_handler(struct irq_data *irq_data)
{
    virq_handle_t virq;
    int err;
    virq = (virq_handle_t)irq_data->token;
    while (virq != NULL) {
        err = vm_inject_IRQ(virq);
        virq = (virq_handle_t)virq->next;
    }
}

static int install_linux_devices(vm_t *vm)
{
    int err;
    int i;
    err = vm_install_ram_only_device(vm, &ram_dev);
    assert(!err);
    err = vm_install_plic(vm);
    assert(!err);


    for (int i = 0; i < ARRAY_SIZE(linux_irq); i++) {
        irq_t irq = linux_irq[i];
#ifdef CONFIG_PER_VCPU_VMM
        if (irq == 133 || irq == 134 || irq == 135) continue;
#endif
        struct irq_data *irq_data = linux_irq_data[irq];
        virq_handle_t virq;
        void (*handler)(struct irq_data *);
        handler = &irq_handler;
        if (irq_data == NULL) {
            printf("register irq %d\n", (int)irq);
            irq_data = irq_server_register_irq(_irq_server, irq, handler, NULL);
            if (!irq_data) {
                printf("fail to register irq %d\n", (int)irq);
                assert(0);
            }
            linux_irq_data[irq] = irq_data;
        }
        virq = vm_virq_new(vm, irq, &do_irq_server_ack, irq_data);
        if (virq == NULL) {
            assert(0);
        }
        virq->next = irq_data->token;
        irq_data->token = (void *)virq;
    }

    return 0;
}



//#define DEBUG_COPYOUT
//#define DEBUG_COPYIN

#ifdef DEBUG_COPYOUT
#define DCOPYOUT(...) printf("copyout: " __VA_ARGS__)
#else
#define DCOPYOUT(...) do{}while(0)
#endif

#ifdef DEBUG_COPYIN
#define DCOPYIN(...) printf("copyin: " __VA_ARGS__)
#else
#define DCOPYIN(...) do{}while(0)
#endif


static int copy_out_page(vspace_t *dst_vspace, vspace_t *src_vspace, vka_t* vka, void* src, void* dst, size_t size)
{
    cspacepath_t dup_cap_path, cap_path;
    seL4_CPtr dup_cap, cap;
    void* tmp_dst;
    int offset;
    size_t copy_size;
    int bits;
    int err;

    /* Create a frame if necessary */
    cap = vspace_get_cap(dst_vspace, dst);
    if (cap == seL4_CapNull) {
        reservation_t res;
        vka_object_t frame;
        bits = MAP_PAGE_BITS;
        /* Create a frame */
        err = vka_alloc_frame(vka, MAP_PAGE_BITS, &frame);
        assert(!err);
        if (err) {
            return -1;
        }
        /* Map the frame to the dest vspace */
        res = vspace_reserve_range_at(dst_vspace, dst, BIT(bits), seL4_AllRights, 1);
        if (!res.res) {
            assert(res.res);
            return -1;
        }
        err = vspace_map_pages_at_vaddr(dst_vspace, &frame.cptr, NULL, dst, 1, bits, res);
        vspace_free_reservation(dst_vspace, res);
        if (err) {
            return -1;
        }
        cap = vspace_get_cap(dst_vspace, dst);
        assert(cap != seL4_CapNull);
    } else {
        bits = (int)vspace_get_cookie(dst_vspace, dst);
        if (bits == 0) {
            bits = MAP_PAGE_BITS;
        }
    }

    /* Copy the cap */
    err = vka_cspace_alloc_path(vka, &dup_cap_path);
    if (err) {
        return -1;
    }
    vka_cspace_make_path(vka, cap, &cap_path);
    dup_cap = dup_cap_path.capPtr;
    err = vka_cnode_copy(&dup_cap_path, &cap_path, seL4_AllRights);
    if (err) {
        vka_cspace_free(vka, dup_cap);
        return -1;
    }

    /* Map it */
    tmp_dst = vspace_map_pages(src_vspace, &dup_cap, NULL, seL4_AllRights, 1, bits, 1);
    assert(tmp_dst);
    if (!tmp_dst) {
        vka_cnode_delete(&dup_cap_path);
        vka_cspace_free(vka, dup_cap);
        return -1;
    }

    /* Copy the data to the frame */
    offset = (uintptr_t)dst & MASK(bits);
    copy_size = BIT(bits) - offset;
    if (copy_size > size) {
        copy_size = size;
    }
    memcpy(tmp_dst + offset, src, copy_size);

    /* Clean up */
    vspace_unmap_pages(src_vspace, tmp_dst, 1, bits, VSPACE_PRESERVE);
    vka_cnode_delete(&dup_cap_path);
    vka_cspace_free(vka, dup_cap);

    DCOPYOUT("copied out page 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dst, copy_size);


    /* Done */
    return copy_size;
}

static int copy_out(vspace_t *dst_vspace, vspace_t *src_vspace, vka_t* vka, void* src, uintptr_t dest, size_t size)
{
    DCOPYOUT("copy out 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dest, size);
    while (size) {
        int seg_size;
        seg_size = copy_out_page(dst_vspace, src_vspace, vka, src, (void*)dest, size);
        assert(seg_size > 0);
        if (seg_size <= 0) {
            return -1;
        }
        dest += seg_size;
        src += seg_size;
        size -= seg_size;
    }
    return 0;
}


static int vm_copyout(vm_t *vm, void *data, uintptr_t address, size_t size)
{
    return copy_out(vm_get_vspace(vm), vm->vmm_vspace, vm->vka, data, address, size);
}

static int copy_in_page(vspace_t *vmm_vspace, vspace_t *vm_vspace, vka_t *vka, void *dest, void *src, size_t size)
{
    seL4_CPtr cap, vmm_cap;
    cspacepath_t cap_path, vmm_cap_path;
    void *tmp_src;
    int offset;
    size_t copy_size;
    int bits;
    int err;

    /* Find the VM frame */
    cap = vspace_get_cap(vm_vspace, src);
    if (cap == seL4_CapNull) {
        printf("null cap %p\n", src);
        return -1;
    }
    bits = vspace_get_cookie(vm_vspace, src);
    if (bits == 0) {
        //bits = 12;
        bits = MAP_PAGE_BITS;
    }
    vka_cspace_make_path(vka, cap, &cap_path);

    /* Copy the cap so that we can map it into the VMM */
    err = vka_cspace_alloc_path(vka, &vmm_cap_path);
    vmm_cap = vmm_cap_path.capPtr;
    if (err) {
        printf("Failed to allocate slot for copyin\n");
        return -1;
    }
    err = vka_cnode_copy(&vmm_cap_path, &cap_path, seL4_AllRights);
    if (err) {
        vka_cspace_free(vka, vmm_cap);
        printf("Failed to copy frame cap for copyin\n");
        return -1;
    }
    /* Map it into the VMM vspace */
    tmp_src = vspace_map_pages(vmm_vspace, &vmm_cap, NULL, seL4_AllRights, 1, bits, 1);
    if (tmp_src == NULL) {
        printf("src %p dest %p\n", src, dest);
        assert(!"Failed to map frame for copyin\n");
        vka_cnode_delete(&vmm_cap_path);
        vka_cspace_free(vka, vmm_cap);
        printf("Failed to map frame cap for copyin\n");
        return -1;
    }

    /* Copy the data from the frame */
    offset = (uintptr_t)src & MASK(bits);
    copy_size = BIT(bits) - offset;
    if (copy_size > size) {
        copy_size = size;
    }
    memcpy(dest, tmp_src + offset, copy_size);

    /* Clean up */
    vspace_unmap_pages(vmm_vspace, tmp_src, 1, bits, VSPACE_PRESERVE);
    vka_cnode_delete(&vmm_cap_path);
    vka_cspace_free(vka, vmm_cap);

    DCOPYIN("copy in page 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dest, copy_size);
    /* Done */
    return copy_size;
}

static int copy_in(vspace_t *dst_vspace, vspace_t *src_vspace, vka_t *vka, void *dest, uintptr_t src, size_t size)
{
    DCOPYIN("copy in 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dest, size);
    while (size) {
        int seg_size;
        seg_size = copy_in_page(dst_vspace, src_vspace, vka, dest, (void*)src, size);
        assert(seg_size > 0);
        if (seg_size <= 0) {
            return -1;
        }
        dest += seg_size;
        src += seg_size;
        size -= seg_size;
    }
    return 0;
}

static int vm_copyin(vm_t *vm, void *data, uintptr_t address, size_t size)
{
    lock(&vm_copyin_lock);
    int ret = copy_in(vm->vmm_vspace, vm_get_vspace(vm), vm->vka, data, address, size);
    unlock(&vm_copyin_lock);
    return ret;
}

#define DTB_ADDR    0x84000000

static void *load_linux(vm_t *vm, const char *kernel_name, const char *dtb_name)
{
    unsigned long size;
    unsigned long cpio_len = _cpio_archive_end - _cpio_archive;
    void *file = cpio_get_file(_cpio_archive, cpio_len, (const char *)kernel_name, &size);
    if (file == NULL) {
        printf("Failed to get %s from CPIO\n", kernel_name);
        return NULL;
    }
    printf("Found %s at %p len %ld %x\n", kernel_name, file, size, *(int *)file);
    seL4_Word entry = RAM_BASE + 0x2000000;
    if (vm_copyout(vm, file, entry, size)) {
        printf("Failed to load %s\n", kernel_name);
        return NULL;
    }

    file = cpio_get_file(_cpio_archive, cpio_len, dtb_name, &size);
    if (file == NULL) {
        printf("Failed to get %s from CPIO\n", dtb_name);
        return NULL;
    }
    if (vm_copyout(vm, file, DTB_ADDR, size)) {
        printf("Failed to load %s\n", dtb_name);
        return NULL;
    }
    return (void *)entry;
}

#define COPY_SIZE  64
static void dump_guest(vm_t *vm, seL4_Word addr)
{
    char buf[COPY_SIZE];
    bzero(buf, COPY_SIZE);
    vm_copyin(vm, &buf, addr, COPY_SIZE);
    int *ptr = (int *)&buf[0];
    for (int i = 0; i < COPY_SIZE / 4; i++) {
        printf("addr %x val %x\n", addr + i * 4, *ptr);
        ptr++;
    }
}

#define GET_PPN(x)      (x & 0xfffffffffff)
#define VPN_MASK        0x1ff
#define VPN_SHIFT(l)    (12 + 9 * (l))
#define GET_VPN(x, l)   (((x) >> VPN_SHIFT(l)) & VPN_MASK)
#define PPN_SHIFT   12
#define PTE_V           BIT(0)
#define PTE_R           BIT(1)
#define PTE_W           BIT(2)
#define PTE_X           BIT(3)
#define PTE_GET_1G(x)   (((x >> 28) & 0x3ffffff))
#define PTE_GET_2M(x)   (((x >> 19) & (BIT(36) - 1)))
#define PTE_GET_4K(x)   (((x >> 10) & (BIT(45) - 1)))
#define SV39_MODE       0x8


//#define DEBUG_PW

#ifdef DEBUG_PW
#define DPW(...)  printf(__VA_ARGS__)
#else
#define DPW(...) do {} while (0)
#endif

static uint64_t pt[CONFIG_MAX_NUM_NODES][512];

/* translate guest va to guest pa Sv39 */
static seL4_Word gva_to_gpa(vm_t *vm, seL4_Word va, int vcpu_id)
{
    int level = 2;
    assert(vcpu_id >=0 && vcpu_id < CONFIG_MAX_NUM_NODES);
    uint64_t *ppt = &pt[vcpu_id][0];
    bzero(ppt, 4096);
    seL4_Word satp = 0;
    seL4_CPtr vcpu = vm_get_vcpu(vm, vcpu_id);
    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SATP);
    assert(!res.error);
    satp = res.value;
    DPW("satp %lx\n", satp);
    assert((satp >> 60) == SV39_MODE);
    seL4_Word ppn = GET_PPN(satp);
    seL4_Word gpa = ppn << PPN_SHIFT;
    uint64_t pte = 0;

    // read in the page table 
    while (level > 0) {
        DPW("copy in %lx level %d\n", gpa, level);
        if (gpa == 0) {
            while (1);
        }
        vm_copyin(vm, ppt, gpa, sizeof(uint64_t) * 512);
        for (int i = 0; i < 512; i++) {
            if (ppt[i] != 0) {
                DPW("pte %d %lx\n", i, ppt[i]);
            }
        }
        int vpn = GET_VPN(va, level);
        pte = ppt[vpn];
        DPW("index %d pte %lx gpa %lx level %d\n", vpn, pte, gpa, level);
        if (pte & PTE_V && (pte & PTE_R || pte & PTE_X)) {
            /* we reach a leaf page */
            if (level == 2) { 
                /* 1 GiB page */
                DPW("pa  %lx %lx level %d\n", PTE_GET_1G(pte), PTE_GET_1G(pte) << 30, level);
                return (PTE_GET_1G(pte) << 30)  | (va & 0x3fffffff);
            }
            if (level == 1) {
                /* 2 MiB page */
                DPW("pa %lx %lx level %d\n", PTE_GET_2M(pte), PTE_GET_2M(pte) << 21, level);


                DPW("return %lx\n", (PTE_GET_2M(pte) << 21) | (va & (BIT(22) - 1)));
                return (PTE_GET_2M(pte) << 21) | (va & (BIT(22) - 1));

            }
            if (level == 0) {
                /* 4 KiB page */
                return ((PTE_GET_4K(pte) << 12) | (va & (BIT(13) - 1)));
            }

        }
        gpa = (pte >> 10) << 12;
        level--;
    }
    DPW("invalid GVA TO GPA translation \n");
    assert(!"invalid");
    return 0;
}

void decode_inst(fault_t *f)
{
    uint32_t *i = &(f->decoded_inst.inst);
    riscv_inst_t *ri = &(f->decoded_inst);
    *i = 0;

    seL4_Word ip_gpa = gva_to_gpa(f->vm, f->ip, f->vcpu_id);
    vm_copyin(f->vm, i, ip_gpa, 4);

    /* with a bit improvement from the kernel, now we do
     * not need to do the nested page table walking to
     * get the faulting instruction for decoding
     */
    f->riscv_inst = *i;
    ri->opcode = OPCODE(*i);
    switch (ri->opcode) {
        case ST_OP:
            ri->rs2 = ST_RS2(*i);
            ri->rs1 = ST_RS1(*i);
            ri->func3 = FUNC3(*i);
            ri->inst = *i;
            break;
        case LD_OP:
            ri->rs1 = LD_RS1(*i);
            ri->rd = LD_RD(*i);
            ri->immi = LD_IMMI(*i);
            ri->func3 = FUNC3(*i);
            break;
        default:
            printf("Invalid op %x\n", ri->opcode);
            break;
    }

    switch (ri->func3) {
        case LD_ST_B:
            f->width = WIDTH_BYTE;
            break;
        case LD_ST_H:
            f->width = WIDTH_HALFWORD;
            break;
        case LD_ST_W:
            f->width = WIDTH_WORD;
            break;
        case LD_ST_D:
            f->width = WIDTH_DOUBLEWORD;
            break;
        default:
            printf("unknonwn store width %d\n", ri->func3);
            break;
    }
    return;
}

#define SBI_SET_TIMER 0
#define SBI_CONSOLE_PUTCHAR 1
#define SBI_CONSOLE_GETCHAR 2
#define SBI_CLEAR_IPI 3
#define SBI_SEND_IPI 4
#define SBI_REMOTE_FENCE_I 5
#define SBI_REMOTE_SFENCE_VMA 6
#define SBI_REMOTE_SFENCE_VMA_ASID 7
#define SBI_SHUTDOWN 8

static int handle_page_fault(vm_t *vm, fault_t *fault)
{
    uintptr_t  addr = fault->addr;
    int err = 0;
    for (int i = 0; i < vm->ndevices; i++) {
        struct device *d = &vm->devices[i];
        assert(d != NULL);
        if (addr >= d->pstart && addr < d->pstart + d->size) {
            err = d->handle_page_fault(d, vm, fault);
            if (err == 0) return 0;
        }
    }
    addr -= (addr & 0xfff);
    printf("Blindly map %lx to %lx\n", addr, addr);
    map_vm_device(vm, addr, addr, seL4_AllRights);
    ignore_fault(fault);
    return err;
}

#define WFI_INST    0x10500073

static int handle_invalid_inst(vm_t *vm, fault_t *fault)
{
    seL4_UserContext *regs = fault_get_ctx(fault);
    seL4_CPtr tcb = fault->vcpu->tcb.cptr;
    if (fault->riscv_inst == WFI_INST) {
        // block the thread by not replying to it, but advance the PC by 4 first
        fault->vcpu->suspended = 1;
        regs->pc += 4;
        seL4_TCB_WriteRegisters(tcb, false, 0,
                sizeof(*regs) / sizeof(regs->pc), regs);
        return 0;
    }

    // ignore other invalid instructions at the moment
    printf("Unhanlded instruction %x pc %lx\n", fault->riscv_inst, regs->pc);

    regs->pc += 4;
    seL4_TCB_WriteRegisters(tcb, false, 0,
            sizeof(*regs) / sizeof(regs->pc), regs);
    restart_fault(fault);

    return 0;

}

#define CAUSE_INVALID_INST  0x2
#define CAUSE_HYPCALL       0xa

static int vm_event(vm_t* vm, seL4_MessageInfo_t tag, seL4_Word badge)
{
    seL4_Word label;
    seL4_Word length;
    int vcpu_id = VM_VCPU_BADGE_GET_VCPU(badge);
    label = seL4_MessageInfo_get_label(tag);
    length = seL4_MessageInfo_get_length(tag);
    fault_t *fault = vm_get_fault(vm, vcpu_id);
    fault->vcpu_id = vcpu_id;
    assert(fault);
    assert(fault->vcpu);
    seL4_CPtr tcb = fault->vcpu->tcb.cptr;
    seL4_CPtr vcpu = fault->vcpu->vcpu.cptr;

#if CONFIG_MAX_NUM_NODES > 1
#ifndef CONFIG_PER_VCPU_VMM
    seL4_TCB_SetAffinity(seL4_CapInitThreadTCB, fault->vcpu->affinity);
#endif
#endif
    switch (label) {

    case seL4_Fault_VMFault: {
        int err;
        err = new_fault(fault);
        assert(!err);
        seL4_UserContext *regs = fault_get_ctx(fault);
        handle_page_fault(vm, fault);
        break;
    }

    case seL4_Fault_UserException: {
        seL4_Word pc;
        int err;
        assert(length == seL4_UserException_Length);

        seL4_UserContext *regs;
        new_user_fault(fault);
        seL4_Word cause = seL4_GetMR(seL4_UserException_Number);
        regs = fault_get_ctx(fault);

        switch (cause) {
            /* invalid instruction */
            case CAUSE_INVALID_INST:
                return handle_invalid_inst(vm, fault);
                break;
            default:
                printf("Unhandled user exception cause %d at pc %lx VCPU ID %d\n",
                        cause, regs->pc, (int)vcpu_id);
                break;
        }

        regs->pc += 4;
        seL4_TCB_WriteRegisters(tcb, false, 0,
                    sizeof(*regs) / sizeof(regs->pc), regs);
        restart_fault(fault);
        return 0;
    }

    break;

    case seL4_Fault_VCPUFault: {
        seL4_MessageInfo_t reply;
        seL4_Word cause;
        int err;
        assert(length == seL4_VCPUFault_Length);
        cause = seL4_GetMR(seL4_VCPUFault_Cause);
        /* check if the exception class (bits 26-31) of the HSR indicate WFI/WFE */
        if (cause == CAUSE_HYPCALL) {
            seL4_UserContext *regs;
            new_fault(fault);
            regs = fault_get_ctx(fault);
            switch (regs->a7) {
                case SBI_CONSOLE_PUTCHAR:
                    putchar(regs->a0);
                    regs->a0 = 0;
                    break;

                case SBI_SET_TIMER: {
                    uint64_t cur = 0;
                    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                    assert(!res.error);
                    res.value &= ~SIP_TIMER;
                    seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, res.value);

                    asm volatile("rdtime %0" : "=r"(cur));
                    if (cur >= regs->a0) {
                        /* Already passed the target time, inject directly */
                        vm_inject_timer_interrupt(vm, fault);
                        break;
                    } 

                    err = seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_TIMER, regs->a0);
                    assert(!err);

                    regs->a0 = 0;
                    break;
                }

                case SBI_SHUTDOWN: {
                    /* Just suspend the VM */
                    int err = seL4_TCB_Suspend(tcb);
                    assert(!err);
                    break;
                }

                case SBI_SEND_IPI: {
                    seL4_Word hartid_mask = regs->a0;
                    seL4_Word ip_gpa = gva_to_gpa(fault->vm, regs->a0, vcpu_id);
                    if (ip_gpa == 0) {
                        printf("regs->a0 is %lx on vcpu %d\n", regs->a0, (int)vcpu_id);
                    }
                    assert(ip_gpa != 0);
                    vm_copyin(fault->vm, &hartid_mask, ip_gpa, sizeof(hartid_mask));
                    //printf("hartmask %llx %d\n", hartid_mask, (int)vcpu_id);
                    for (int i = 0; i < MAX_NUM_VCPUS; i++) {
                        if (hartid_mask & BIT(i)) {
#ifdef CONFIG_PER_VCPU_VMM
                            if (i == vcpu_id) {
                                seL4_CPtr vcpu = vm_get_vcpu(vm, i);
                                seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                                assert(!res.error);
                                res.value |= BIT(1);
                                seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, res.value);
                            } else {
                                seL4_MessageInfo_t info = seL4_MessageInfo_new(IPI_MESSAGE_LABEL, 0, 0, 1);
                                seL4_SetMR(0, i);
                                seL4_NBSend(vm->vcpus[i].fault_ep, info);
                            }
#else
#if CONFIG_MAX_NUM_NODES > 1
                            seL4_TCB_SetAffinity(seL4_CapInitThreadTCB, i);
#endif

                            seL4_CPtr vcpu = vm_get_vcpu(vm, i);

                            seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                            assert(!res.error);
                            res.value |= BIT(1);
                            seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, res.value);
#endif

                        }
                    }
                    regs->a0 = 0;
                    break;
                }

                case SBI_CLEAR_IPI: {
                    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                    assert(!res.error);
                    res.value &= ~BIT(1);
                    seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, res.value);
                    regs->a0 = 0;
                    break;
                }

                default:
                    break;
                    printf("unhandled hypcall %d\n", regs->a7);
            }

            regs->pc += 4;
            seL4_TCB_WriteRegisters(tcb, false, 0,
                        sizeof(*regs) / sizeof(regs->pc), regs);
            restart_fault(fault);
            return 0;
        }
        return -1;
    }

    break;
    default:
        /* What? Why are we here? What just happened? */
        printf("Unknown fault from [%s]: label=%p length=%p\n",
               vm->name, (void *) label, (void *) length);
        return -1;
    }
    return 0;
}

int main(void)
{
    vm_t vm;
    int err;
    err = vmm_init();
    assert(!err);

    /* Create the VM */
    err = vm_create(VM_NAME, VM_PRIO, _fault_endpoint, VM_BADGE,
                    &_vka, &_simple, &_vspace, &_io_ops, &vm);
    _irq_server->vm = &vm;

    if (err) {
        printf("Failed to create VM\n");
        seL4_DebugHalt();
        return -1;
    }

    err = install_linux_devices(&vm);
    assert(!err);
    /* Load system images */
    printf("Loading Linux: \'%s\' dtb: \'%s\'\n", VM_LINUX_NAME, VM_LINUX_DTB_NAME);
    void *entry = load_linux(&vm, VM_LINUX_NAME, VM_LINUX_DTB_NAME);
    if (entry == NULL) {
        printf("Failed to load VM image\n");
        seL4_DebugHalt();
        return -1;
    }
    vm_set_bootargs(&vm, (seL4_Word)entry, 0, DTB_ADDR); 
    /* Power on */
    printf("Starting VM\n\n");

    err = vm_start(&vm);

#if CONFIG_MAX_NUM_NODES > 1
#if CONFIG_MAX_NUM_NODES >= 2
    vm_add_vcpu(&vm, (seL4_Word)entry, 1, DTB_ADDR);
#endif
#if CONFIG_MAX_NUM_NODES >= 3
    vm_add_vcpu(&vm, (seL4_Word)entry, 2, DTB_ADDR);
#endif
#if CONFIG_MAX_NUM_NODES >= 4
    vm_add_vcpu(&vm, (seL4_Word)entry, 3, DTB_ADDR);
#endif
#endif

    if (err) {
        printf("Failed to start VM\n");
        seL4_DebugHalt();
        return -1;
    }

    /* Loop forever, handling events */
    while (1) {
        seL4_MessageInfo_t tag;
        seL4_Word sender_badge;

        tag = seL4_Recv(_fault_endpoint, &sender_badge);
        if (sender_badge == 0) {
            seL4_Word label;
            label = seL4_MessageInfo_get_label(tag);
            switch (label) {
                case IRQ_MESSAGE_LABEL:
                irq_server_handle_irq_ipc(_irq_server);
                break;

                case IPI_MESSAGE_LABEL: {
                    seL4_Word mr = seL4_GetMR(0);
                    assert(mr == 0);
                    seL4_CPtr vcpu = vm_get_vcpu(&vm, 0);
                    seL4_RISCV_VCPU_ReadRegs_t res = seL4_RISCV_VCPU_ReadRegs(vcpu, seL4_VCPUReg_SIP);
                    assert(!res.error);
                    res.value |= BIT(1);
                    seL4_RISCV_VCPU_WriteRegs(vcpu, seL4_VCPUReg_SIP, res.value);
                    vcpu_info_t *vi = &vm.vcpus[0];
                    if (vi->suspended) {
                        restart_fault(vi->fault);
                        vi->suspended = 0;
                    }
                    break;
                }
                default:
                    printf("Unknown label (%d) for IPC badge %d\n", label, sender_badge);
                    break;
            }

        } else {
            err = vm_event(&vm, tag, sender_badge);
            if (err) {
                /* Shutdown */
                vm_stop(&vm);
                seL4_DebugHalt();
                while (1);
            }
        }
    }

    return 0;
}
