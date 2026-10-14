/*
Memory configuration for the QEMU raspi2b machine (BCM2836, 1 GiB RAM at 0).

QEMU puts its boot stubs in the first pages of RAM, so start the image at 0x80000.
*/

MEMORY {
    RAM : ORIGIN = 0x00080000, LENGTH = 64M
}

REGION_ALIAS("VECTORS", RAM);
REGION_ALIAS("CODE", RAM);
REGION_ALIAS("DATA", RAM);
REGION_ALIAS("STACKS", RAM);

PROVIDE(_sys_stack_size = 256K);
