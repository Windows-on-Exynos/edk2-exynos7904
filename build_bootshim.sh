cd BootShim
make UEFI_BASE=0x90000000 UEFI_SIZE=0x00b30000
rm BootShim.elf
cd ..