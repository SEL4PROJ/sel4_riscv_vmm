#!/bin/bash
qemu-system-riscv64 -machine virt -cpu rv64,h=true -nographic -serial mon:stdio -m size=4095M -kernel images/sel4-riscv-vmm-image-riscv-spike

