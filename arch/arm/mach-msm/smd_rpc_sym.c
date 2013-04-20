/* Autogenerated by mkrpcsym.pl.  Do not edit */



#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/module.h>

struct sym {
	const char *str;
};

const char *smd_rpc_syms[] = {
	"CB CM_FUSION",				/*0x30010000*/
	"CB DB",				/*0x30000001*/
	"CB SND",				/*0x30000002*/
	"CB WMS_FUSION",			/*0x30010003*/
	"CB PDSM",				/*0x30000004*/
	"CB MISC_MODEM_APIS",			/*0x30000005*/
	"CB MISC_APPS_APIS",			/*0x30000006*/
	"CB JOYST",				/*0x30000007*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB ADSPRTOSATOM",			/*0x3000000A*/
	"CB ADSPRTOSMTOA",			/*0x3000000B*/
	"CB I2C",				/*0x3000000C*/
	"CB TIME_REMOTE",			/*0x3000000D*/
	"CB NV_FUSION",				/*0x3001000E*/
	"CB CLKRGM_SEC_FUSION",			/*0x3001000F*/
	"CB RDEVMAP",				/*0x30000010*/
	"CB UNDEFINED",
	"CB PBMLIB_FUSION",			/*0x30010012*/
	"CB AUDMGR",				/*0x30000013*/
	"CB MVS",				/*0x30000014*/
	"CB DOG_KEEPALIVE",			/*0x30000015*/
	"CB GSDI_EXP_FUSION",			/*0x30010016*/
	"CB AUTH",				/*0x30000017*/
	"CB NVRUIMI",				/*0x30000018*/
	"CB MMGSDILIB_FUSION",			/*0x30010019*/
	"CB CHARGER",				/*0x3000001A*/
	"CB UIM_FUSION",			/*0x3001001B*/
	"CB UNDEFINED",
	"CB PDSM_ATL",				/*0x3000001D*/
	"CB FS_XMOUNT",				/*0x3000001E*/
	"CB SECUTIL",				/*0x3000001F*/
	"CB MCCMEID",				/*0x30000020*/
	"CB PM_STROBE_FLASH",			/*0x30000021*/
	"CB UNDEFINED",
	"CB SMD_BRIDGE",			/*0x30000023*/
	"CB SMD_PORT_MGR_FUSION",		/*0x30010024*/
	"CB BUS_PERF",				/*0x30000025*/
	"CB BUS_MON_REMOTE",			/*0x30000026*/
	"CB MC",				/*0x30000027*/
	"CB MCCAP",				/*0x30000028*/
	"CB MCCDMA",				/*0x30000029*/
	"CB MCCDS",				/*0x3000002A*/
	"CB MCCSCH",				/*0x3000002B*/
	"CB MCCSRID",				/*0x3000002C*/
	"CB SNM",				/*0x3000002D*/
	"CB MCCSYOBJ",				/*0x3000002E*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB DSRLP_APIS",			/*0x30000031*/
	"CB RLP_APIS",				/*0x30000032*/
	"CB DS_MP_SHIM_MODEM",			/*0x30000033*/
	"CB UNDEFINED",
	"CB DSHDR_MDM_APIS",			/*0x30000035*/
	"CB DS_MP_SHIM_APPS",			/*0x30000036*/
	"CB HDRMC_APIS",			/*0x30000037*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB PMAPP_OTG",				/*0x3000003A*/
	"CB DIAG",				/*0x3000003B*/
	"CB GSTK_EXP_FUSION",			/*0x3001003C*/
	"CB DSBC_MDM_APIS",			/*0x3000003D*/
	"CB HDRMRLP_MDM_APIS",			/*0x3000003E*/
	"CB UNDEFINED",
	"CB HDRMC_MRLP_APIS",			/*0x30000040*/
	"CB PDCOMM_APP_API",			/*0x30000041*/
	"CB DSAT_APIS",				/*0x30000042*/
	"CB RFM",				/*0x30000043*/
	"CB CMIPAPP",				/*0x30000044*/
	"CB DSMP_UMTS_MODEM_APIS",		/*0x30000045*/
	"CB UNDEFINED",
	"CB DSUCSDMPSHIM",			/*0x30000047*/
	"CB TIME_REMOTE_ATOM",			/*0x30000048*/
	"CB UNDEFINED",
	"CB SD",				/*0x3000004A*/
	"CB MMOC",				/*0x3000004B*/
	"CB UNDEFINED",
	"CB WLAN_CP_CM",			/*0x3000004D*/
	"CB FTM_WLAN",				/*0x3000004E*/
	"CB UNDEFINED",
	"CB CPRMINTERFACE",			/*0x30000050*/
	"CB DATA_ON_MODEM_MTOA_APIS",		/*0x30000051*/
	"CB UNDEFINED",
	"CB MISC_MODEM_APIS_NONWINMOB",		/*0x30000053*/
	"CB MISC_APPS_APIS_NONWINMOB",		/*0x30000054*/
	"CB PMEM_REMOTE",			/*0x30000055*/
	"CB TCXOMGR",				/*0x30000056*/
	"CB UNDEFINED",
	"CB BT",				/*0x30000058*/
	"CB PD_COMMS_API",			/*0x30000059*/
	"CB PD_COMMS_CLIENT_API",		/*0x3000005A*/
	"CB PDAPI",				/*0x3000005B*/
	"CB UNDEFINED",
	"CB TIME_REMOTE_MTOA",			/*0x3000005D*/
	"CB FTM_BT",				/*0x3000005E*/
	"CB DSUCSDAPPIF_APIS",			/*0x3000005F*/
	"CB PMAPP_GEN",				/*0x30000060*/
	"CB PM_LIB_FUSION",			/*0x30010061*/
	"CB UNDEFINED",
	"CB HSU_APP_APIS",			/*0x30000063*/
	"CB HSU_MDM_APIS",			/*0x30000064*/
	"CB ADIE_ADC_REMOTE_ATOM",		/*0x30000065*/
	"CB TLMM_REMOTE_ATOM",			/*0x30000066*/
	"CB UI_CALLCTRL",			/*0x30000067*/
	"CB UIUTILS",				/*0x30000068*/
	"CB PRL",				/*0x30000069*/
	"CB HW",				/*0x3000006A*/
	"CB OEM_RAPI_FUSION",			/*0x3001006B*/
	"CB WMSPM",				/*0x3000006C*/
	"CB BTPF",				/*0x3000006D*/
	"CB UNDEFINED",
	"CB USB_APPS_RPC",			/*0x3000006F*/
	"CB USB_MODEM_RPC",			/*0x30000070*/
	"CB ADC",				/*0x30000071*/
	"CB CAMERAREMOTED",			/*0x30000072*/
	"CB SECAPIREMOTED",			/*0x30000073*/
	"CB DSATAPI",				/*0x30000074*/
	"CB CLKCTL_RPC",			/*0x30000075*/
	"CB BREWAPPCOORD",			/*0x30000076*/
	"CB UNDEFINED",
	"CB WLAN_TRP_UTILS",			/*0x30000078*/
	"CB GPIO_RPC",				/*0x30000079*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB L1_DS",				/*0x3000007C*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB OSS_RRCASN_REMOTE",			/*0x3000007F*/
	"CB PMAPP_OTG_REMOTE",			/*0x30000080*/
	"CB PING_LTE_RPC",			/*0x30010081*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UKCC_IPC_APIS",			/*0x30000087*/
	"CB UNDEFINED",
	"CB VBATT_REMOTE",			/*0x30000089*/
	"CB MFPAL_FPS",				/*0x3000008A*/
	"CB DSUMTSPDPREG",			/*0x3000008B*/
	"CB LOC_API",				/*0x3000008C*/
	"CB UNDEFINED",
	"CB CMGAN",				/*0x3000008E*/
	"CB ISENSE",				/*0x3000008F*/
	"CB TIME_SECURE",			/*0x30000090*/
	"CB HS_REM",				/*0x30000091*/
	"CB ACDB",				/*0x30000092*/
	"CB NET",				/*0x30000093*/
	"CB LED",				/*0x30000094*/
	"CB DSPAE",				/*0x30000095*/
	"CB MFKAL",				/*0x30000096*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB TEST_API",				/*0x3000009B*/
	"CB REMOTEFS_SRV_API_FUSION",		/*0x3001009C*/
	"CB ISI_TRANSPORT",			/*0x3000009D*/
	"CB OEM_FTM",				/*0x3000009E*/
	"CB TOUCH_SCREEN_ADC",			/*0x3000009F*/
	"CB SMD_BRIDGE_APPS_FUSION",		/*0x300100A0*/
	"CB SMD_BRIDGE_MODEM_FUSION",		/*0x300100A1*/
	"CB OEM_RAPI_COMMON_PIPE",		/*0x300200A2*/
	"CB OEM_RAPI_FCLK",			/*0x300200A3*/
	"CB NPA_REMOTE",			/*0x300000A4*/
	"CB MMGSDISESSIONLIB_FUSION",		/*0x300100A5*/
	"CB IFTA_REMOTE",			/*0x300000A6*/
	"CB REMOTE_STORAGE",			/*0x300000A7*/
	"CB MF_REMOTE_FILE",			/*0x300000A8*/
	"CB MFSC_CHUNKED_TRANSPORT",		/*0x300000A9*/
	"CB MFIM3",				/*0x300000AA*/
	"CB FM_WAN_API",			/*0x300000AB*/
	"CB WLAN_RAPI",				/*0x300000AC*/
	"CB DSMGR_APIS",			/*0x300000AD*/
	"CB CM_MM_FUSION",			/*0x300100AE*/
};

static struct sym_tbl {
	const char **data;
	int size;
} tbl = { smd_rpc_syms, ARRAY_SIZE(smd_rpc_syms)};

const char *smd_rpc_get_sym(uint32_t val)
{
	int idx = val & 0xFFFF;
	if (idx < tbl.size) {
		if (val & 0x01000000)
			return tbl.data[idx];
		else
			return tbl.data[idx] + 3;
	}
	return 0;
}
EXPORT_SYMBOL(smd_rpc_get_sym);

