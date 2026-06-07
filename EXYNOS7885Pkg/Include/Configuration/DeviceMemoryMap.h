#ifndef _DEVICE_MEMORY_MAP_H_
#define _DEVICE_MEMORY_MAP_H_

#include <Library/ArmLib.h>

#define MAX_ARM_MEMORY_REGION_DESCRIPTOR_COUNT 128

/* Below flag is used for system memory */
#define SYSTEM_MEMORY_RESOURCE_ATTR_CAPABILITIES                               \
  EFI_RESOURCE_ATTRIBUTE_PRESENT | EFI_RESOURCE_ATTRIBUTE_INITIALIZED |        \
      EFI_RESOURCE_ATTRIBUTE_TESTED | EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE |     \
      EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE |                               \
      EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE |                         \
      EFI_RESOURCE_ATTRIBUTE_WRITE_BACK_CACHEABLE |                            \
      EFI_RESOURCE_ATTRIBUTE_EXECUTION_PROTECTABLE

typedef enum { NoHob, AddMem, AddDev, HobOnlyNoCacheSetting, MaxMem } DeviceMemoryAddHob;

#define MEMORY_REGION_NAME_MAX_LENGTH 32

typedef struct {
  CHAR8                        Name[MEMORY_REGION_NAME_MAX_LENGTH];
  EFI_PHYSICAL_ADDRESS         Address;
  UINT64                       Length;
  DeviceMemoryAddHob           HobOption;
  EFI_RESOURCE_TYPE            ResourceType;
  EFI_RESOURCE_ATTRIBUTE_TYPE  ResourceAttribute;
  EFI_MEMORY_TYPE              MemoryType;
  ARM_MEMORY_REGION_ATTRIBUTES ArmAttributes;
} ARM_MEMORY_REGION_DESCRIPTOR_EX, *PARM_MEMORY_REGION_DESCRIPTOR_EX;

#define MEM_RES EFI_RESOURCE_MEMORY_RESERVED
#define MMAP_IO EFI_RESOURCE_MEMORY_MAPPED_IO
#define SYS_MEM EFI_RESOURCE_SYSTEM_MEMORY

#define SYS_MEM_CAP SYSTEM_MEMORY_RESOURCE_ATTR_CAPABILITIES
#define INITIALIZED EFI_RESOURCE_ATTRIBUTE_INITIALIZED
#define WRITE_COMBINEABLE EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE
#define UNCACHEABLE EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE

#define Reserv EfiReservedMemoryType
#define Conv EfiConventionalMemory
#define BsData EfiBootServicesData
#define BsCode EfiBootServicesCode
#define RtData EfiRuntimeServicesData
#define RtCode EfiRuntimeServicesCode
#define MmIO EfiMemoryMappedIO

#define DEVICE ARM_MEMORY_REGION_ATTRIBUTE_DEVICE
#define WRITE_THROUGH ARM_MEMORY_REGION_ATTRIBUTE_WRITE_THROUGH
#define WRITE_THROUGH_XN ARM_MEMORY_REGION_ATTRIBUTE_WRITE_THROUGH
#define WRITE_BACK ARM_MEMORY_REGION_ATTRIBUTE_WRITE_BACK
#define WRITE_BACK_XN ARM_MEMORY_REGION_ATTRIBUTE_WRITE_BACK
#define UNCACHED_UNBUFFERED ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED
#define UNCACHED_UNBUFFERED_XN ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED

static ARM_MEMORY_REGION_DESCRIPTOR_EX gDeviceMemoryDescriptorEx[] = {
/*                                                    EFI_RESOURCE_ EFI_RESOURCE_ATTRIBUTE_ EFI_MEMORY_TYPE ARM_REGION_ATTRIBUTE_
     MemLabel(32 Char.),  MemBase,    MemSize, BuildHob, ResourceType, ResourceAttribute, MemoryType, CacheAttributes
*/

//--------------------- DDR --------------------- */

    {"HLOS 0",             0x80000000, 0x00C00000, AddMem, SYS_MEM, SYS_MEM_CAP, Conv, WRITE_BACK_XN},
    {"UEFI Stack",         0x80C00000, 0x00040000, AddMem, SYS_MEM, SYS_MEM_CAP,  BsData, WRITE_BACK},
    {"CPU Vectors",        0x80C40000, 0x00010000, AddMem, SYS_MEM, SYS_MEM_CAP,  BsCode, WRITE_BACK},
    {"HLOS 1",             0x80C50000, 0x133B0000, AddMem, SYS_MEM, SYS_MEM_CAP, Conv,   WRITE_BACK},
    {"UEFI FD",            0x94000000, 0x00200000, AddMem, SYS_MEM, SYS_MEM_CAP, BsCode, WRITE_BACK},
    {"HLOS 1.5",           0x94200000, 0x2BA00000, AddMem, SYS_MEM, SYS_MEM_CAP, Conv,   WRITE_BACK},
    /*Memory hole 0xbbc00000 -> 0xc0000000*/
    {"HLOS 3",             0xBFC00000, 0x2D400000, AddMem, SYS_MEM, SYS_MEM_CAP,  Conv,   WRITE_BACK},
    {"Display Reserved",   0xED000000, 0x01400000, AddMem, MEM_RES, SYS_MEM_CAP, Reserv, WRITE_THROUGH_XN},
    {"HLOS 4",             0xEE400000, 0x13800000, AddMem, SYS_MEM, SYS_MEM_CAP,  Conv,   WRITE_BACK},

//--------------------- Register ---------------------
    {"Chip Info",          0x10000000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Pinctrl SHUB",       0x11080000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Pinctrl ALIVE",      0x11850000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"PMU",                0x11860000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Speedy",             0x11A10000, 0x00002000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Pinctrl CMGP",       0x11C20000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Clock Controller",   0x12100000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Gic Distributor",    0x12301000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Gic Redistributor",  0x12302000, 0x00006000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"CMU FSYS",           0x13400000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Sysreg FSYS",        0x13410000, 0x00002000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Pinctrl FSYS",       0x13490000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"DW MMC0",            0x13500000, 0x00002000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"UFS UniPro",         0x13510000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"UFS HCI",            0x13520000, 0x00005000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"DW MMC2",            0x13550000, 0x00002000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Pinctrl TOP",        0x139B0000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Decon",              0x148B0000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},
	{"Pinctrl DISPAUD",    0x14A60000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   DEVICE},

    /* Terminator for MMU */
    { "Terminator", 0, 0, 0, 0, 0, 0, 0}};

#endif