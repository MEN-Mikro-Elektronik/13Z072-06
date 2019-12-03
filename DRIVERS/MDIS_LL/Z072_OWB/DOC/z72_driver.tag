<?xml version='1.0' encoding='ISO-8859-1' standalone='yes'?>
<tagfile>
  <compound kind="page">
    <filename>index</filename>
    <title></title>
    <name>index</name>
  </compound>
  <compound kind="file">
    <name>z72_doc.c</name>
    <path>/opt/menlinux/DRIVERS/MDIS_LL/Z072_OWB/DRIVER/COM/</path>
    <filename>z72__doc_8c</filename>
  </compound>
  <compound kind="file">
    <name>z72_drv.c</name>
    <path>/opt/menlinux/DRIVERS/MDIS_LL/Z072_OWB/DRIVER/COM/</path>
    <filename>z72__drv_8c</filename>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_Init</name>
      <anchor>a1</anchor>
      <arglist>(DESC_SPEC *descSpec, OSS_HANDLE *osHdl, MACCESS *ma, OSS_SEM_HANDLE *devSemHdl, OSS_IRQ_HANDLE *irqHdl, LL_HANDLE **llHdlP)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_Exit</name>
      <anchor>a2</anchor>
      <arglist>(LL_HANDLE **llHdlP)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_Read</name>
      <anchor>a3</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 ch, int32 *value)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_Write</name>
      <anchor>a4</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 ch, int32 value)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_SetStat</name>
      <anchor>a5</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 ch, int32 code, INT32_OR_64 value32_or_64)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_GetStat</name>
      <anchor>a6</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 ch, int32 code, INT32_OR_64 *value32_or_64P)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_BlockRead</name>
      <anchor>a7</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 ch, void *buf, int32 size, int32 *nbrRdBytesP)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_BlockWrite</name>
      <anchor>a8</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 ch, void *buf, int32 size, int32 *nbrWrBytesP)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_Irq</name>
      <anchor>a9</anchor>
      <arglist>(LL_HANDLE *llHdl)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Z72_Info</name>
      <anchor>a10</anchor>
      <arglist>(int32 infoType,...)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>char *</type>
      <name>Ident</name>
      <anchor>a11</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>Cleanup</name>
      <anchor>a12</anchor>
      <arglist>(LL_HANDLE *llHdl, int32 retCode)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>readRom</name>
      <anchor>a13</anchor>
      <arglist>(LL_HANDLE *llHdl, u_int8 *buf, u_int32 numBytes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>skipRom</name>
      <anchor>a14</anchor>
      <arglist>(LL_HANDLE *llHdl)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>readMemory</name>
      <anchor>a15</anchor>
      <arglist>(LL_HANDLE *llHdl, u_int32 majState, u_int8 *buf, u_int16 size, u_int16 offs)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>masterTxReset</name>
      <anchor>a16</anchor>
      <arglist>(LL_HANDLE *llHdl)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>waitOnReady</name>
      <anchor>a17</anchor>
      <arglist>(LL_HANDLE *llHdl)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>getDevError</name>
      <anchor>a18</anchor>
      <arglist>(LL_HANDLE *llHdl)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int32</type>
      <name>execCmd</name>
      <anchor>a19</anchor>
      <arglist>(LL_HANDLE *llHdl, u_int8 *data, u_int32 cmd)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>byteCrc</name>
      <anchor>a20</anchor>
      <arglist>(u_int8 *crc, u_int8 c)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>byteCrcFinish</name>
      <anchor>a21</anchor>
      <arglist>(u_int8 *crc)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>u_int8</type>
      <name>bufCrc8</name>
      <anchor>a22</anchor>
      <arglist>(u_int8 crcStart, u_int8 *p, u_int32 len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>Z72_GetEntry</name>
      <anchor>a23</anchor>
      <arglist>(LL_ENTRY *drvP)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>u_int8</type>
      <name>reflectByte</name>
      <anchor>a24</anchor>
      <arglist>(u_int8 c)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>const char</type>
      <name>IdentString</name>
      <anchor>a0</anchor>
      <arglist>[]</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>z72_drv_int.h</name>
    <path>/opt/menlinux/DRIVERS/MDIS_LL/Z072_OWB/DRIVER/COM/</path>
    <filename>z72__drv__int_8h</filename>
    <class kind="struct">LL_HANDLE</class>
    <member kind="define">
      <type>#define</type>
      <name>Z72_REG_WRT</name>
      <anchor>a13</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_REG_RD</name>
      <anchor>a14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_REG_CTL</name>
      <anchor>a15</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_REG_STS</name>
      <anchor>a16</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_MASK</name>
      <anchor>a17</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_WBIT</name>
      <anchor>a18</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_RBIT</name>
      <anchor>a19</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_RPP</name>
      <anchor>a20</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_PRGP</name>
      <anchor>a21</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_WBYT</name>
      <anchor>a22</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_CMD_RBYT</name>
      <anchor>a23</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_EXEC</name>
      <anchor>a24</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_MIRQ</name>
      <anchor>a25</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_CTL_EIRQ</name>
      <anchor>a26</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_EEXE</name>
      <anchor>a27</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_ERPP</name>
      <anchor>a28</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_EPP1</name>
      <anchor>a29</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_EPP2</name>
      <anchor>a30</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_MIRQ</name>
      <anchor>a31</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_EIRQ</name>
      <anchor>a32</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_BUSY</name>
      <anchor>a33</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_STS_PRES</name>
      <anchor>a34</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>Z72_STATE_MAJOR</name>
      <anchor>a76</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_IDLE</name>
      <anchor>a76a50</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_RP_PP</name>
      <anchor>a76a51</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_ROM_SKIP</name>
      <anchor>a76a52</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_ROM_READ</name>
      <anchor>a76a53</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_ROM_MATCH</name>
      <anchor>a76a54</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_ROM_SEARCH</name>
      <anchor>a76a55</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_IDLE_MEM</name>
      <anchor>a76a56</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_MEM_READ</name>
      <anchor>a76a57</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_MEM_READ_CRC</name>
      <anchor>a76a58</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_MEM_WRITE</name>
      <anchor>a76a59</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_STATUS_READ</name>
      <anchor>a76a60</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_STATUS_WRITE</name>
      <anchor>a76a61</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_MAJ_END</name>
      <anchor>a76a62</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>Z72_STATE_READ</name>
      <anchor>a77</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_RX_CMD</name>
      <anchor>a77a63</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_RX_TA</name>
      <anchor>a77a64</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_RX_CRC1</name>
      <anchor>a77a65</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_RX_DATA</name>
      <anchor>a77a66</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_RX_CRC2</name>
      <anchor>a77a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_RX_END</name>
      <anchor>a77a68</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>Z72_STATE_WRITE</name>
      <anchor>a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_CMD</name>
      <anchor>a78a69</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_TA</name>
      <anchor>a78a70</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_DATA</name>
      <anchor>a78a71</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_CRC</name>
      <anchor>a78a72</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_PGMP</name>
      <anchor>a78a73</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_DATA_VER</name>
      <anchor>a78a74</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Z72_ST_TX_END</name>
      <anchor>a78a75</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>_NO_LL_HANDLE</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CH_NUMBER</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USE_IRQ</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ADDRSPACE_COUNT</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ADDRSPACE_SIZE</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_POLL_TOUT</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_POLL_RETRY_MAX</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_SEM_TOUT</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_RETRY_MAX</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_EXTRADBG</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DBG_MYLEVEL</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DBH</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OSH</name>
      <anchor>a12</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_MODE_POLL</name>
      <anchor>a35</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_MODE_AUTO_SKIPROM</name>
      <anchor>a36</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_ORDER</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_HIGHBIT</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_POLY</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_INIT</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_READ</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_MATCH</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_SEARCH</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_SKIP</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_MEM_READ</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_MEM_READ_CRC</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_MEM_WRITE</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_STATUS_READ</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_STATUS_WRITE</name>
      <anchor>a12</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>LL_HANDLE</name>
    <filename>structLL__HANDLE.html</filename>
    <member kind="variable">
      <type>int32</type>
      <name>memAlloc</name>
      <anchor>o0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OSS_HANDLE *</type>
      <name>osHdl</name>
      <anchor>o1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OSS_IRQ_HANDLE *</type>
      <name>irqHdl</name>
      <anchor>o2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>DESC_HANDLE *</type>
      <name>descHdl</name>
      <anchor>o3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>MACCESS</type>
      <name>ma</name>
      <anchor>o4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>MDIS_IDENT_FUNCT_TBL</type>
      <name>idFuncTbl</name>
      <anchor>o5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int32</type>
      <name>dbgLevel</name>
      <anchor>o6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>DBG_HANDLE *</type>
      <name>dbgHdl</name>
      <anchor>o7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int32</type>
      <name>mode</name>
      <anchor>o8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int32</type>
      <name>irqCount</name>
      <anchor>o9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int32</type>
      <name>exeStatus</name>
      <anchor>o10</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OSS_SEM_HANDLE *</type>
      <name>exeSem</name>
      <anchor>o11</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OSS_SEM_HANDLE *</type>
      <name>devSemHdl</name>
      <anchor>o12</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int8 *</type>
      <name>dataP</name>
      <anchor>o13</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int32</type>
      <name>majState</name>
      <anchor>o14</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u_int32</type>
      <name>fncState</name>
      <anchor>o15</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>dummy</name>
    <title></title>
    <filename>dummy</filename>
  </compound>
  <compound kind="group">
    <name>_Z72_DS2502</name>
    <title>OWB / DS2502 HW specific defines</title>
    <filename>group____Z72__DS2502.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_ORDER</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_HIGHBIT</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_POLY</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROMCRC_INIT</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_READ</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_MATCH</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_SEARCH</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_ROM_SKIP</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_MEM_READ</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_MEM_READ_CRC</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_MEM_WRITE</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_STATUS_READ</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z72_OWB_DS2502_STATUS_WRITE</name>
      <anchor>a12</anchor>
      <arglist></arglist>
    </member>
  </compound>
</tagfile>
