cd BootShim
make UEFI_BASE=0x94000000 UEFI_SIZE=0x00b50000
rm BootShim.elf
cd ..