typedef u64 RmPhysAddr;
typedef u64 NvLength;
typedef u64 NvU64;
typedef u64 NvP64;

typedef u32 NvHandle;
typedef u32 NvU32;
typedef u32 NvV32;

typedef s32 NvS32;

typedef u16 NvU16;

typedef u8 NvU8;
typedef u8 NvBool;

typedef struct GspSMInfo_t
{
    NvU32 version;
    NvU32 regBankCount;
    NvU32 regBankRegCount;
    NvU32 maxWarpsPerSM;
    NvU32 maxThreadsPerWarp;
    NvU32 geomGsObufEntries;
    NvU32 geomXbufEntries;
    NvU32 maxSPPerSM;
    NvU32 rtCoreCount;
} GspSMInfo;

#define NV0080_CTRL_GR_CAPS_TBL_SIZE            23

/* maximum possible number of bytes of GID information returned */
#define NV2080_GPU_MAX_GID_LENGTH        (0x000000100)

typedef struct NV2080_CTRL_GPU_GET_GID_INFO_PARAMS {
    NvU32 index;
    NvU32 flags;
    NvU32 length;
    NvU8  data[NV2080_GPU_MAX_GID_LENGTH];
} NV2080_CTRL_GPU_GET_GID_INFO_PARAMS;

typedef struct NV2080_CTRL_GPU_GET_FERMI_GPC_INFO_PARAMS {
    NvU32 gpcMask;
} NV2080_CTRL_GPU_GET_FERMI_GPC_INFO_PARAMS;

typedef struct NV2080_CTRL_GPU_GET_FERMI_TPC_INFO_PARAMS {
    NvU32 gpcId;
    NvU32 tpcMask;
} NV2080_CTRL_GPU_GET_FERMI_TPC_INFO_PARAMS;

#define MAX_GPC_COUNT           32

typedef struct NV2080_CTRL_GPU_GET_FERMI_ZCULL_INFO_PARAMS {
    NvU32 gpcId;
    NvU32 zcullMask;
} NV2080_CTRL_GPU_GET_FERMI_ZCULL_INFO_PARAMS;

typedef struct NV2080_CTRL_BIOS_GET_SKU_INFO_PARAMS {
    NvU32 BoardID;
    char  chipSKU[4];
    char  chipSKUMod[2];
    char  project[5];
    char  projectSKU[5];
    char  CDP[6];
    char  projectSKUMod[2];
    NvU32 businessCycle;
} NV2080_CTRL_BIOS_GET_SKU_INFO_PARAMS;

#define NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_MEM_TYPES   17

typedef NvBool NV2080_CTRL_CMD_FB_GET_FB_REGION_SURFACE_MEM_TYPE_FLAG[NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_MEM_TYPES];

typedef struct NV2080_CTRL_CMD_FB_GET_FB_REGION_FB_REGION_INFO {
    NvU64 base;
    NvU64 limit;
    NvU64 reserved;
    NvU32                                                  performance;
    NvBool                                                 supportCompressed;
    NvBool                                                 supportISO;
    NvBool                                                 bProtected;
    NV2080_CTRL_CMD_FB_GET_FB_REGION_SURFACE_MEM_TYPE_FLAG blackList;
} NV2080_CTRL_CMD_FB_GET_FB_REGION_FB_REGION_INFO;

#define NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_MAX_ENTRIES 16

typedef struct NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_PARAMS {
    NvU32 numFBRegions;
    NV2080_CTRL_CMD_FB_GET_FB_REGION_FB_REGION_INFO fbRegion[NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_MAX_ENTRIES];
} NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_PARAMS;

typedef enum
{
    COMPUTE_BRANDING_TYPE_NONE,
    COMPUTE_BRANDING_TYPE_TESLA,
} COMPUTE_BRANDING_TYPE;

typedef struct NV0080_CTRL_GPU_GET_SRIOV_CAPS_PARAMS {
    NvU32  totalVFs;
    NvU32  firstVfOffset;
    NvU32  vfFeatureMask;
    NvU64 FirstVFBar0Address;
    NvU64 FirstVFBar1Address;
    NvU64 FirstVFBar2Address;
    NvU64 bar0Size;
    NvU64 bar1Size;
    NvU64 bar2Size;
    NvBool b64bitBar0;
    NvBool b64bitBar1;
    NvBool b64bitBar2;
    NvBool bSriovEnabled;
    NvBool bSriovHeavyEnabled;
    NvBool bEmulateVFBar0TlbInvalidationRegister;
    NvBool bClientRmAllocatedCtxBuffer;
} NV0080_CTRL_GPU_GET_SRIOV_CAPS_PARAMS;

typedef enum NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS {
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_MAIN = 0,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_SPILL = 1,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_PAGEPOOL = 2,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_BETACB = 3,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_RTV = 4,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_CONTEXT_POOL = 5,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_CONTEXT_POOL_CONTROL = 6,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_CONTEXT_POOL_CONTROL_CPU = 7,
    NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_END = 8,
} NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS;

#define NV2080_GPU_MAX_NAME_STRING_LENGTH                  (0x0000040)

typedef struct VIRTUAL_DISPLAY_GET_MAX_RESOLUTION_PARAMS 
{
    NvU32 headIndex;
    NvU32 maxHResolution;
    NvU32 maxVResolution;
} VIRTUAL_DISPLAY_GET_MAX_RESOLUTION_PARAMS;

typedef struct VIRTUAL_DISPLAY_GET_NUM_HEADS_PARAMS 
{
    NvU32 numHeads;
    NvU32 maxNumHeads;
} VIRTUAL_DISPLAY_GET_NUM_HEADS_PARAMS;

// Fetched from GSP-RM into CPU-RM
typedef struct GspStaticConfigInfo_t
{
    NvU8 grCapsBits[NV0080_CTRL_GR_CAPS_TBL_SIZE];
    NV2080_CTRL_GPU_GET_GID_INFO_PARAMS gidInfo;
    NV2080_CTRL_GPU_GET_FERMI_GPC_INFO_PARAMS gpcInfo;
    NV2080_CTRL_GPU_GET_FERMI_TPC_INFO_PARAMS tpcInfo[MAX_GPC_COUNT];
    NV2080_CTRL_GPU_GET_FERMI_ZCULL_INFO_PARAMS zcullInfo[MAX_GPC_COUNT];
    NV2080_CTRL_BIOS_GET_SKU_INFO_PARAMS SKUInfo;
    NV2080_CTRL_CMD_FB_GET_FB_REGION_INFO_PARAMS fbRegionInfoParams;
    COMPUTE_BRANDING_TYPE computeBranding;

    NV0080_CTRL_GPU_GET_SRIOV_CAPS_PARAMS sriovCaps;
    NvU32 sriovMaxGfid;

    NvU64 engineCaps;

    GspSMInfo SM_info;

    NvBool poisonFuseEnabled;
  
    NvU64 fb_length;
    NvU32 fbio_mask;
    NvU32 fb_bus_width;
    NvU32 fb_ram_type;
    NvU32 fbp_mask;
    NvU32 l2_cache_size;

    NvU32 gfxpBufferSize[NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_CONTEXT_POOL];
    NvU32 gfxpBufferAlignment[NV2080_CTRL_CMD_GR_CTXSW_PREEMPTION_BIND_BUFFERS_CONTEXT_POOL];

    NvU8 gpuNameString[NV2080_GPU_MAX_NAME_STRING_LENGTH];
    NvU8 gpuShortNameString[NV2080_GPU_MAX_NAME_STRING_LENGTH];
    NvU16 gpuNameString_Unicode[NV2080_GPU_MAX_NAME_STRING_LENGTH];
    NvBool bGpuInternalSku;
    NvBool bIsQuadroGeneric;
    NvBool bIsQuadroAd;
    NvBool bIsNvidiaNvs;
    NvBool bIsVgx;
    NvBool bGeforceSmb;
    NvBool bIsTitan;
    NvBool bIsTesla;

    NvU64 bar1PdeBase;
    NvU64 bar2PdeBase;

    NvBool bVbiosValid;
    NvU32 vbiosSubVendor;
    NvU32 vbiosSubDevice;

    NvBool bPageRetirementSupported;

    NvBool bSplitVasBetweenServerClientRm;

    NvBool bClRootportNeedsNosnoopWAR;

    VIRTUAL_DISPLAY_GET_NUM_HEADS_PARAMS displaylessMaxHeads;
    VIRTUAL_DISPLAY_GET_MAX_RESOLUTION_PARAMS displaylessMaxResolution;
    NvU64 displaylessMaxPixels;

    // Client handle for internal RMAPI control.
    NvHandle hInternalClient;

    // Device handle for internal RMAPI control.
    NvHandle hInternalDevice;

    // Subdevice handle for internal RMAPI control.
    NvHandle hInternalSubdevice;
} GspStaticConfigInfo;
