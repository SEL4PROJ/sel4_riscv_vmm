#!/bin/bash
qemu-system-riscv64 -bios none -machine virt -cpu rv64,x-h=true -nographic -serial mon:stdio -m size=4095M -kernel images/sel4-riscv-vmm-image-riscv-spike

