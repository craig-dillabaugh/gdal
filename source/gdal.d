/*
 * D Bindings for the Geospatial Data Abstraction Library (GDAL) C Library
 * Version 1.11.0
 *
 * Binding Author: Craig Dillabaugh
 *
 * For more information on GDAL itself see:  http://www.gdal.org/
 *
 *
 * Current build command:
 *     dmd test_gdal_d.d gdal.d -L-lgdal
 */
 module gdal;
 
 import std.c.stdio;
 
 /*
  * The enum CPLErr is defined in the header file cpl_error.h. I have not
  * for the time being included a binding to that header, but have just 
  * imported this single symbol from it.
  *
  * Similarly, GDALProgressFunc is defined in port/cpl_progress.h
  */
enum CPLErr
{
    CE_None = 0,
    CE_Debug = 1,
    CE_Warning = 2,
    CE_Failure = 3,
    CE_Fatal = 4
}

extern(C) alias 
GDALProgressFunc = int function( double dfComplete, const(char)* pszMessage, 
				 void* pProgressArg );
				 
alias GIntBig = long; //GIntBig is a 64-bit integer and long is always 
		      //64 bits in D
alias GUIntBig = ulong;

alias GInt16 = short;
alias GUInt16 = ushort;

alias GByte = ubyte;  //GByte is unsigned char in C.
alias GBool = int;


extern(C) enum CPLXMLNodeType {  //From cpl_minixml.h
    CXT_Element = 0,
    CXT_Text = 1,
    CXT_Attribute = 2,
    CXT_Comment = 3,
    CXT_Literal = 4
}

extern(C) struct CPLXMLNode  //From cpl_minixml.h
{
    CPLXMLNodeType      eType;
    char*               pszValue;
    CPLXMLNode*         psNext; 
    CPLXMLNode*         psChild;
}



/* ******************* END of CPL Symbols ********************** /
 
 /*! Pixel data types */
enum GDALDataType {
    GDT_Unknown = 0,
    GDT_Byte = 1,
    GDT_UInt16 = 2,
    GDT_Int16 = 3,
    GDT_UInt32 = 4,
    GDT_Int32 = 5,
    GDT_Float32 = 6,
    GDT_Float64 = 7,
    GDT_CInt16 = 8,
    GDT_CInt32 = 9,
    GDT_CFloat32 = 10,
    GDT_CFloat64 = 11,
    GDT_TypeCount = 12
}
 
extern(C) int GDALGetDataTypeSize( GDALDataType );
extern(C) int GDALDataTypeIsComplex( GDALDataType );
extern(C) const(char)* GDALGetDataTypeName( GDALDataType );
extern(C) GDALDataType GDALGetDataTypeByName( const(char) * );
extern(C) GDALDataType GDALDataTypeUnion( GDALDataType, GDALDataType );

enum GDALAsyncStatusType
{	
	GARIO_PENDING = 0,
	GARIO_UPDATE = 1,
	GARIO_ERROR = 2,
	GARIO_COMPLETE = 3,
	GARIO_TypeCount = 4
}

extern(C) const(char)* GDALGetAsyncStatusTypeName( GDALAsyncStatusType );
extern(C) GDALAsyncStatusType GDALGetAsyncStatusTypeByName( const(char)* );

/*! Flag indicating read/write, or read-only access to data. */
enum GDALAccess{
    GA_ReadOnly = 0,
    GA_Update = 1
}

/*! Read/Write flag for RasterIO() method */
enum GDALRWFlag{
    GF_Read = 0,
    GF_Write = 1
}

enum GDALRIOResampleAlg {
    GRIORA_NearestNeighbour = 0,                  
    GRIORA_Bilinear = 1,                        
    GRIORA_Cubic = 2, 
    GRIORA_CubicSpline = 3,
    GRIORA_Lanczos = 4,
    GRIORA_Average = 5,                                
    GRIORA_Mode = 6,
    GRIORA_Gauss = 7                              
}

enum RASTERIO_EXTRA_ARG_CURRENT_VERSION = 1;

struct GDALRasterIOExtraArg
{
    int                 nVersion;
    GDALRIOResampleAlg  eResampleAlg;
    GDALProgressFunc    pfnProgress;
    void*		pProgressData;
    int                 bFloatingPointWindowValidity;
    double              dfXOff;
    double              dfYOff;
    double              dfXSize;
    double              dfYSize;
}

//This replaces the INIT_RASTERIO_EXTRA_ARGs Macro.
void INIT_RASTERIO_EXTRA_ARG( ref GDALRasterIOExtraArg s )
{
  s.nVersion = RASTERIO_EXTRA_ARG_CURRENT_VERSION;
  s.eResampleAlg = GDALRIOResampleAlg.GRIORA_NearestNeighbour;
  s.pfnProgress = null;
  s.pProgressData = null;
  s.bFloatingPointWindowValidity = 0;
}

/*! Types of color interpretation for raster bands. */
enum GDALColorInterp
{
    GCI_Undefined=0,
    GCI_GrayIndex=1,
    GCI_PaletteIndex=2,
    GCI_RedBand=3,
    GCI_GreenBand=4,
    GCI_BlueBand=5,
    GCI_AlphaBand=6,
    GCI_HueBand=7,    
    GCI_SaturationBand=8,
    GCI_LightnessBand=9,
    GCI_CyanBand=10,
    GCI_MagentaBand=11,
    GCI_YellowBand=12,
    GCI_BlackBand=13,
    GCI_YCbCr_YBand=14,
    GCI_YCbCr_CbBand=15,
    GCI_YCbCr_CrBand=16,
    GCI_Max=16
}

extern(C) const(char)* GDALGetColorInterpretationName( GDALColorInterp );
extern(C) GDALColorInterp GDALGetColorInterpretationByName( const(char)* pszName );

extern(C) const(char)* GDALGetColorInterpretationName( GDALColorInterp );
extern(C) GDALColorInterp  GDALGetColorInterpretationByName( const(char)*pszName );

enum GDALPaletteInterp 
{
  GPI_Gray=0,
  GPI_RGB=1,
  GPI_CMYK=2,
  GPI_HLS=3
}

extern(C) const(char)* GDALGetPaletteInterpretationName( GDALPaletteInterp );

enum string GDALMD_AREA_OR_POINT = "AREA_OR_POINT"; 
enum string GDALMD_AOP_AREA      = "Area";
enum string GDALMD_AOP_POINT     = "Point";

enum int CPLE_WrongFormat  = 200;

alias int*  SAFile;
alias void* GDALMajorObjectH;
alias void* GDALDatasetH;
alias void* GDALRasterBandH;
alias void* GDALDriverH;



alias void* GDALColorTableH;
alias void* GDALRasterAttributeTableH;
alias void* GDALAsyncReaderH;

alias GSpacing = GIntBig;

enum string GDAL_DMD_LONGNAME 		= "DMD_LONGNAME";
enum string GDAL_DMD_HELPTOPIC 		= "DMD_HELPTOPIC";
enum string GDAL_DMD_MIMETYPE 		= "DMD_MIMETYPE";
enum string GDAL_DMD_EXTENSION 		= "DMD_EXTENSION";
enum string GDAL_DMD_CONNECTION_PREFIX  = "DMD_CONNECTION_PREFIX";
enum string GDAL_DMD_CREATIONOPTIONLIST = "DMD_CREATIONOPTIONLIST"; 
enum string GDAL_DMD_CREATIONDATATYPES 	= "DMD_CREATIONDATATYPES";
enum string GDAL_DMD_SUBDATASETS 	= "DMD_SUBDATASETS"; 
enum string GDAL_DCAP_OPEN       	= "DCAP_OPEN";
enum string GDAL_DCAP_CREATE 		= "DCAP_CREATE";
enum string GDAL_DCAP_CREATECOPY	= "DCAP_CREATECOPY";
enum string GDAL_DCAP_VIRTUALIO		= "DCAP_VIRTUALIO";
enum string GDAL_DCAP_RASTER     	= "DCAP_RASTER";
enum string GDAL_DCAP_VECTOR            = "DCAP_VECTOR";
enum string GDAL_DCAP_NOTNULL_FIELDS 	= "DCAP_NOTNULL_FIELDS";
enum string GDAL_DCAP_DEFAULT_FIELDS 	= "DCAP_DEFAULT_FIELDS";
enum string GDAL_DCAP_NOTNULL_GEOMFIELDS = "DCAP_NOTNULL_GEOMFIELDS"; 

extern(C) void GDALAllRegister();

extern(C) GDALDatasetH 
GDALCreate(   GDALDriverH hDriver,
	      const(char)*, int, int, int, GDALDataType,
	      char** );

//TODO: What to do with CPL_WARN_UNUSED RESULT ????
//gdal-1.11.0/port/cpl_port.h (l 571-575) This is a Macro that append
// __attribute__((warn_unused_result))  for GCC compilers > version 4
//and where DOxygen is not turned off.  I think it is likely safe to
//ignore.
extern(C) GDALDatasetH
GDALCreateCopy( GDALDriverH, const(char)*, GDALDatasetH,
                int, char**, GDALProgressFunc, void* );

extern(C) GDALDriverH 
GDALIdentifyDriver( const(char)* pszFilename,
                    char** papszFileList );

extern(C) GDALDatasetH
GDALOpen( const(char)*pszFilename, GDALAccess eAccess );

extern(C) GDALDatasetH
GDALOpenShared( const(char)*, GDALAccess );

enum int GDAL_OF_READONLY      = 0x00; 
enum int GDAL_OF_UPDATE        = 0x01; 
enum int GDAL_OF_ALL  	       = 0x00; 
enum int GDAL_OF_RASTER        = 0x02;
enum int GDAL_OF_VECTOR        = 0x04;
enum int GDAL_OF_KIND_MASK     = 0x1E;
enum int GDAL_OF_SHARED        = 0x20;
enum int GDAL_OF_VERBOSE_ERROR = 0x40;
enum int GDAL_OF_INTERNAL      = 0x80;

extern(C) GDALDatasetH GDALOpenEx( const(char)* pszFilename,
				  uint nOpenFlags,
				  const(char*)  papszAllowedDrivers,
				  const(char*)  papszOpenOptions,
				  const(char*)  papszSiblingFiles );

extern(C) int GDALDumpOpenDatasets( FILE * );

extern(C) GDALDriverH GDALGetDriverByName( const(char) * );
extern(C) int  GDALGetDriverCount( );
extern(C) GDALDriverH GDALGetDriver( int );
extern(C) void GDALDestroyDriver( GDALDriverH );
extern(C) int  GDALRegisterDriver( GDALDriverH );
extern(C) void GDALDeregisterDriver( GDALDriverH );
extern(C) void GDALDestroyDriverManager();
extern(C) void GDALDestory();
extern(C) CPLErr GDALDeleteDataset( GDALDriverH, const(char)* );
extern(C) CPLErr GDALRenameDataset( GDALDriverH, 
                                    const(char)* pszNewName,
                                    const(char)* pszOldName );
                                    
extern(C) CPLErr GDALCopyDatasetFiles( GDALDriverH, 
                                       const(char)* pszNewName,
                                       const(char)* pszOldName);
                                       
extern(C) int GDALValidateCreationOptions( GDALDriverH,
                                           char** papszCreationOptions);

/* Deprecated functions omitted, including:
 *
 * GDALGetDriverShortName
 * GDALGetDriverLongName
 * GDALGetDriverHelpTopic
 * GDALGetDriverCreationOptionList
 *
 */
 
/** Ground Control Point */
struct GDAL_GCP
{
    char*       pszId; 
    char*       pszInfo;
    double      dfGCPPixel;
    double      dfGCPLine;
    double      dfGCPX;
    double      dfGCPY;
    double      dfGCPZ;
}

extern(C) void GDALInitGCPs( int, GDAL_GCP* );
extern(C) void GDALDeinitGCPs( int, GDAL_GCP* );
extern(C) GDAL_GCP* GDALDuplicateGCPs( int, const(GDAL_GCP)* );

extern(C) int
GDALGCPsToGeoTransform( int nGCPCount, const(GDAL_GCP)* pasGCPs, 
                        double *padfGeoTransform, int bApproxOK ); 

extern(C) int 
GDALInvGeoTransform( double* padfGeoTransformIn, 
		     double* padfInvGeoTransformOut );
                     

extern(C) void GDALApplyGeoTransform( double*, double, double, 
                                      double*, double* );

extern(C) void GDALComposeGeoTransforms(const(double)* padfGeoTransform1,
                                        const(double)* padfGeoTransform2,
                                        double* padfGeoTransformOut);
                                        
/* major objects (dataset, and, driver, drivermanager).            */

extern(C) char**  GDALGetMetadataDomainList( GDALMajorObjectH hObject );
extern(C) char**  GDALGetMetadata( GDALMajorObjectH, const(char)* );
extern(C) CPLErr  GDALSetMetadata( GDALMajorObjectH, char**,
                                            const(char)* );
extern(C) const(char)*  
GDALGetMetadataItem( GDALMajorObjectH, const(char)*, const(char)* );

extern(C) CPLErr
GDALSetMetadataItem( GDALMajorObjectH, const(char)*, const(char)*,
                     const(char)* );

extern(C) const(char)* GDALGetDescription( GDALMajorObjectH );
extern(C) void GDALSetDescription( GDALMajorObjectH, const(char)* );


enum string GDAL_DS_LAYER_CREATIONOPTIONLIST = "DS_LAYER_CREATIONOPTIONLIST";

/*  GDALDataset class ... normally this represents one file. */

extern(C) GDALDriverH   GDALGetDatasetDriver( GDALDatasetH );
extern(C) char**	GDALGetFileList( GDALDatasetH );
extern(C) void     	GDALClose( GDALDatasetH );
extern(C) int       	GDALGetRasterXSize( GDALDatasetH );
extern(C) int       	GDALGetRasterYSize( GDALDatasetH );
extern(C) int       	GDALGetRasterCount( GDALDatasetH );
extern(C) GDALRasterBandH GDALGetRasterBand( GDALDatasetH, int );

extern(C) CPLErr GDALAddBand( GDALDatasetH hDS, GDALDataType eType, 
			      char **papszOptions );

extern(C) GDALAsyncReaderH 
GDALBeginAsyncReader(GDALDatasetH hDS, int nXOff, int nYOff,
                     int nXSize, int nYSize,
                     void* pBuf, int nBufXSize, int nBufYSize,
                     GDALDataType eBufType, int nBandCount, int* panBandMap,
                     int nPixelSpace, int nLineSpace, int nBandSpace,
                     char** papszOptions);

extern(C) void
GDALEndAsyncReader(GDALDatasetH hDS, GDALAsyncReaderH hAsynchReaderH);

extern(C) CPLErr   GDALDatasetRasterIO( 
    GDALDatasetH hDS, GDALRWFlag eRWFlag,
    int nDSXOff, int nDSYOff, int nDSXSize, int nDSYSize,
    void* pBuffer, int nBXSize, int nBYSize, GDALDataType eBDataType,
    int nBandCount, int* panBandCount, 
    int nPixelSpace, int nLineSpace, int nBandSpace);
    
    
extern(C) CPLErr GDALDatasetRasterIOEx( 
    GDALDatasetH hDS, GDALRWFlag eRWFlag,
    int nDSXOff, int nDSYOff, int nDSXSize, int nDSYSize,
    void* pBuffer, int nBXSize, int nBYSize, GDALDataType eBDataType,
    int nBandCount, int *panBandCount, 
    GSpacing nPixelSpace, GSpacing nLineSpace, GSpacing nBandSpace,
    GDALRasterIOExtraArg* psExtraArg);

extern(C) CPLErr   GDALDatasetAdviseRead( GDALDatasetH hDS, 
    int nDSXOff, int nDSYOff, int nDSXSize, int nDSYSize,
    int nBXSize, int nBYSize, GDALDataType eBDataType,
    int nBandCount, int* panBandCount, char** papszOptions );

extern(C) const(char)*  GDALGetProjectionRef( GDALDatasetH );
extern(C) CPLErr   GDALSetProjection( GDALDatasetH, const(char)* );
extern(C) CPLErr   GDALGetGeoTransform( GDALDatasetH, double* );
extern(C) CPLErr   GDALSetGeoTransform( GDALDatasetH, double* );

extern(C) int    GDALGetGCPCount( GDALDatasetH );
extern(C) const(char)*  GDALGetGCPProjection( GDALDatasetH );
extern(C) const(GDAL_GCP)* GDALGetGCPs( GDALDatasetH );
extern(C) CPLErr GDALSetGCPs( GDALDatasetH, int, const(GDAL_GCP)*,
                              const(char)* );

extern(C) void* GDALGetInternalHandle( GDALDatasetH, const(char)* );
extern(C) int   GDALReferenceDataset( GDALDatasetH );
extern(C) int   GDALDereferenceDataset( GDALDatasetH );

extern(C) CPLErr  
GDALBuildOverviews( GDALDatasetH, const(char)*, int, int*,
                    int, int*, GDALProgressFunc, void* );

extern(C) void  GDALGetOpenDatasets( GDALDatasetH** hDS, int *pnCount );
extern(C) int   GDALGetAccess(  GDALDatasetH hDS );
extern(C) void  GDALFlushCache( GDALDatasetH hDS );

extern(C) CPLErr   
GDALCreateDatasetMaskBand( GDALDatasetH hDS, int nFlags );

extern(C) CPLErr   
GDALDatasetCopyWholeRaster( GDALDatasetH hSrcDS, GDALDatasetH hDstDS, 
			    char** papszOptions, GDALProgressFunc pfnProgress, 
			    void*  pProgressData );

extern(C) CPLErr 
GDALRasterBandCopyWholeRaster( GDALRasterBandH hSrcBand, GDALRasterBandH hDstBand, 
			       char** papszOptions, GDALProgressFunc pfnProgress, 
			       void* pProgressData );

extern(C) CPLErr  
GDALRegenerateOverviews( GDALRasterBandH hSrcBand, 
                         int nOverviewCount, GDALRasterBandH *pahOverviewBands,
                         const(char)* pszResampling, 
                         GDALProgressFunc pfnProgress, void *pProgressData );


extern(C) int        GDALDatasetGetLayerCount( GDALDatasetH );
extern(C) OGRLayerH  GDALDatasetGetLayer( GDALDatasetH, int );
extern(C) OGRLayerH  GDALDatasetGetLayerByName( GDALDatasetH, const(char)* );
extern(C) OGRErr     GDALDatasetDeleteLayer( GDALDatasetH, int );
extern(C) OGRLayerH  GDALDatasetCreateLayer( GDALDatasetH, const(char)*, 
                                      OGRSpatialReferenceH, OGRwkbGeometryType,
                                      char ** );
extern(C) OGRLayerH  GDALDatasetCopyLayer( GDALDatasetH, OGRLayerH, const(char)*,
                                        char ** );
extern(C) int        GDALDatasetTestCapability( GDALDatasetH, const(char)* );
extern(C) OGRLayerH  GDALDatasetExecuteSQL( GDALDatasetH, const(char)*,
                                     OGRGeometryH, const(char)* );
extern(C) void   GDALDatasetReleaseResultSet( GDALDatasetH, OGRLayerH );
extern(C) OGRStyleTableH  GDALDatasetGetStyleTable( GDALDatasetH );
extern(C) void   GDALDatasetSetStyleTableDirectly( GDALDatasetH, OGRStyleTableH );
extern(C) void   GDALDatasetSetStyleTable( GDALDatasetH, OGRStyleTableH );
extern(C) OGRErr GDALDatasetStartTransaction(GDALDatasetH hDS, int bForce);
extern(C) OGRErr GDALDatasetCommitTransaction(GDALDatasetH hDS);
extern(C) OGRErr GDALDatasetRollbackTransaction(GDALDatasetH hDS);                         
                         
/*      GDALRasterBand ... one band/channel in a dataset.               */

/*
void someFunc(void *arg) { printf("Called someFunc!\n"); }  // C code
typedef void (*Callback)(void *);
extern "C" Callback getCallback(void)
{
    return someFunc;
}
extern(C) alias Callback = int function(int, int);  // D code
extern(C) Callback getCallback();
void main()
{
    Callback cb = getCallback();
    cb();  // invokes the callback
}
*/
extern(C) alias 
GDALDerivedPixelFunc = CPLErr function( void** papoSources, int nSources, void *pData,
			int nBufXSize, int nBufYSize,
			GDALDataType eSrcType, GDALDataType eBufType,
                        int nPixelSpace, int nLineSpace);

extern(C) GDALDataType   GDALGetRasterDataType( GDALRasterBandH );

extern(C) void GDALGetBlockSize( GDALRasterBandH, int* pnXSize, int* pnYSize );

extern(C) CPLErr 
GDALRasterAdviseRead( GDALRasterBandH hRB, 
    int nDSXOff, int nDSYOff, int nDSXSize, int nDSYSize,
    int nBXSize, int nBYSize, GDALDataType eBDataType, char** papszOptions );

extern(C) CPLErr   
GDALRasterIO( GDALRasterBandH hRBand, GDALRWFlag eRWFlag,
              int nDSXOff, int nDSYOff, int nDSXSize, int nDSYSize,
              void* pBuffer, int nBXSize, int nBYSize,GDALDataType eBDataType,
              int nPixelSpace, int nLineSpace );
              
extern(C) CPLErr 
GDALRasterIOEx( GDALRasterBandH hRBand, GDALRWFlag eRWFlag,
              int nDSXOff, int nDSYOff, int nDSXSize, int nDSYSize,
              void * pBuffer, int nBXSize, int nBYSize,GDALDataType eBDataType,
              GSpacing nPixelSpace, GSpacing nLineSpace,
              GDALRasterIOExtraArg* psExtraArg );
              
extern(C) CPLErr GDALReadBlock( GDALRasterBandH, int, int, void* );
extern(C) CPLErr GDALWriteBlock( GDALRasterBandH, int, int, void* );
extern(C) int GDALGetRasterBandXSize( GDALRasterBandH );
extern(C) int GDALGetRasterBandYSize( GDALRasterBandH );
extern(C) GDALAccess GDALGetRasterAccess( GDALRasterBandH );
extern(C) int GDALGetBandNumber( GDALRasterBandH );
extern(C) GDALDatasetH GDALGetBandDataset( GDALRasterBandH );

extern(C) GDALColorInterp  
GDALGetRasterColorInterpretation( GDALRasterBandH );

extern(C) CPLErr   
GDALSetRasterColorInterpretation( GDALRasterBandH, GDALColorInterp );

extern(C) GDALColorTableH GDALGetRasterColorTable( GDALRasterBandH );

extern(C) CPLErr GDALSetRasterColorTable( GDALRasterBandH, GDALColorTableH );

extern(C) int GDALHasArbitraryOverviews( GDALRasterBandH );
extern(C) int GDALGetOverviewCount( GDALRasterBandH );
extern(C) GDALRasterBandH GDALGetOverview( GDALRasterBandH, int );
extern(C) double GDALGetRasterNoDataValue( GDALRasterBandH, int* );
extern(C) CPLErr GDALSetRasterNoDataValue( GDALRasterBandH, double );
extern(C) char** GDALGetRasterCategoryNames( GDALRasterBandH );
extern(C) CPLErr GDALSetRasterCategoryNames( GDALRasterBandH, char** );
extern(C) double GDALGetRasterMinimum( GDALRasterBandH, int* pbSuccess );
extern(C) double GDALGetRasterMaximum( GDALRasterBandH, int* pbSuccess );
extern(C) CPLErr GDALGetRasterStatistics( 
	    GDALRasterBandH, int bApproxOK, int bForce, 
	    double* pdfMin, double* pdfMax, double* pdfMean, double* pdfStdDev );
	    
extern(C) CPLErr 
GDALComputeRasterStatistics( 
    GDALRasterBandH, int bApproxOK, 
    double *pdfMin, double *pdfMax, double *pdfMean, double *pdfStdDev,
    GDALProgressFunc pfnProgress, void *pProgressData );

extern(C) CPLErr 
GDALSetRasterStatistics( 
    GDALRasterBandH hBand, 
    double dfMin, double dfMax, double dfMean, double dfStdDev );

extern(C) const(char)* GDALGetRasterUnitType( GDALRasterBandH );

extern(C) CPLErr GDALSetRasterUnitType( GDALRasterBandH hBand, const(char)* pszNewValue );
extern(C) double GDALGetRasterOffset( GDALRasterBandH, int* pbSuccess );
extern(C) CPLErr GDALSetRasterOffset( GDALRasterBandH hBand, double dfNewOffset);
extern(C) double GDALGetRasterScale( GDALRasterBandH, int* pbSuccess );
extern(C) CPLErr GDALSetRasterScale( GDALRasterBandH hBand, double dfNewOffset );

extern(C) void   
GDALComputeRasterMinMax( GDALRasterBandH hBand, int bApproxOK,
                         double[2] adfMinMax );
extern(C) CPLErr GDALFlushRasterCache( GDALRasterBandH hBand );

deprecated( "Use GDALGetRasterHistogramEx() instead") {
  extern(C) CPLErr 
  GDALGetRasterHistogram( GDALRasterBandH hBand,
                          double dfMin, double dfMax,
                          int nBuckets, int* panHistogram,
                          int bIncludeOutOfRange, int bApproxOK,
                          GDALProgressFunc pfnProgress,
                          void* pProgressData );
}

deprecated( "Use GDALGetDefaultHistogramEx() instead") {
  extern(C) CPLErr
  GDALGetDefaultHistogram( GDALRasterBandH hBand,
			   double* pdfMin, double* pdfMax,
			   int* pnBuckets, int** ppanHistogram,
                           int bForce, GDALProgressFunc pfnProgress,
                           void *pProgressData );
}

extern(C) CPLErr 
GDALGetRasterHistogramEx( GDALRasterBandH hBand,
                          double dfMin, double dfMax,
                          int nBuckets, GUIntBig *panHistogram,
                          int bIncludeOutOfRange, int bApproxOK,
                          GDALProgressFunc pfnProgress,
                          void * pProgressData );
                                        
extern(C) CPLErr 
GDALGetDefaultHistogramEx( GDALRasterBandH hBand,
                           double *pdfMin, double *pdfMax,
                           int *pnBuckets, GUIntBig **ppanHistogram,
                           int bForce,
                           GDALProgressFunc pfnProgress,
                           void * pProgressData );

deprecated( "Use GDALSetDefaultHistogramEx() instead") {
  extern(C) CPLErr 
  GDALSetDefaultHistogram( GDALRasterBandH hBand,
                           double dfMin, double dfMax,
                           int nBuckets, int* panHistogram );
}

extern(C) CPLErr 
GDALSetDefaultHistogramEx( GDALRasterBandH hBand,
                           double dfMin, double dfMax,
                           int nBuckets, GUIntBig *panHistogram );


extern(C) int  
GDALGetRandomRasterSample( GDALRasterBandH, int, float* );

extern(C) GDALRasterBandH
GDALGetRasterSampleOverview( GDALRasterBandH, int );

extern(C) GDALRasterBandH
GDALGetRasterSampleOverviewEx( GDALRasterBandH, GUIntBig );

extern(C) CPLErr 
GDALFillRaster( GDALRasterBandH hBand,
                double dfRealValue, double dfImaginaryValue );

extern(C) CPLErr  
GDALComputeBandStats( GDALRasterBandH hBand, int nSampleStep, 
                      double* pdfMean, double *pdfStdDev, 
                      GDALProgressFunc pfnProgress,
                      void* pProgressData );

extern(C) CPLErr 
GDALOverviewMagnitudeCorrection( GDALRasterBandH hBaseBand, 
                                 int nOverviewCount, 
                                 GDALRasterBandH* pahOverviews, 
                                 GDALProgressFunc pfnProgress, 
                                 void* pProgressData );

extern(C) GDALRasterAttributeTableH
GDALGetDefaultRAT( GDALRasterBandH hBand );
    
extern(C) CPLErr GDALSetDefaultRAT( GDALRasterBandH, 
                                    GDALRasterAttributeTableH );
                                    
extern(C) CPLErr 
GDALAddDerivedBandPixelFunc( const(char)* pszName,
                             GDALDerivedPixelFunc pfnPixelFunc );

extern(C) GDALRasterBandH GDALGetMaskBand( GDALRasterBandH hBand );
extern(C) int GDALGetMaskFlags( GDALRasterBandH hBand );

extern(C) CPLErr   
GDALCreateMaskBand( GDALRasterBandH hBand, int nFlags );

enum int GMF_ALL_VALID 		= 0x01;
enum int GMF_PER_DATASET    	= 0x02;
enum int GMF_ALPHA         	= 0x04;
enum int GMF_NODATA        	= 0x08;

/* GDALAsyncReader */

extern(C) GDALAsyncStatusType   
GDALARGetNextUpdatedRegion(GDALAsyncReaderH hARIO, double dfTimeout,
                           int* pnXBufOff,  int* pnYBufOff, 
                           int* pnXBufSize, int* pnYBufSize );

extern(C) int GDALARLockBuffer(GDALAsyncReaderH hARIO,
                               double dfTimeout);

extern(C) void GDALARUnlockBuffer(GDALAsyncReaderH hARIO); 

/* -------------------------------------------------------------------- */
/*      Helper functions.                                               */
/* -------------------------------------------------------------------- */
extern(C) int GDALGeneralCmdLineProcessor( int nArgc, char*** ppapszArgv, 
                                           int nOptions );
                                   
extern(C) void GDALSwapWords( void *pData, int nWordSize, int nWordCount,
                              int nWordSkip );
extern(C) void   
GDALCopyWords( void* pSrcData, GDALDataType eSrcType, int nSrcPixelOffset,
               void* pDstData, GDALDataType eDstType, int nDstPixelOffset,
               int nWordCount );

extern(C) void  
GDALCopyBits( const(GByte)* pabySrcData, int nSrcOffset, int nSrcStep, 
              GByte* pabyDstData, int nDstOffset, int nDstStep,
              int nBitCount, int nStepCount );

extern(C) int GDALLoadWorldFile( const(char)*, double * );
extern(C) int GDALReadWorldFile( const(char)*, const(char)*,
                                 double* );
                                 
extern(C) int GDALWriteWorldFile( const(char)*, const(char)*,
                                  double * );
extern(C) int GDALLoadTabFile( const(char)*, double*, char**,
                               int* , GDAL_GCP** );
extern(C) int GDALReadTabFile( const(char)*, double*, char**,
                               int*, GDAL_GCP** );
extern(C) int GDALLoadOziMapFile( const(char)*, double*, char**,
                                  int*, GDAL_GCP** );
extern(C) int GDALReadOziMapFile( const(char)*, double*,
                                  char**, int*, GDAL_GCP** );                                
                                  

extern(C) const(char)* GDALDecToDMS( double, const(char)*, int );
extern(C) double GDALPackedDMSToDec( double );
extern(C) double GDALDecToPackedDMS( double );

/* Note to developers : please keep this section in sync with ogr_core.h */

//#ifndef GDAL_VERSION_INFO_DEFINED
//#define GDAL_VERSION_INFO_DEFINED
extern(C) const(char)* GDALVersionInfo( const(char) * );
//#endif

//#ifndef GDAL_CHECK_VERSION

extern(C) int GDALCheckVersion( int nVersionMajor, int nVersionMinor,
                                const(char)* pszCallingComponentName);

/** Helper macro for GDALCheckVersion()
  @see GDALCheckVersion()
  */
//#define GDAL_CHECK_VERSION(pszCallingComponentName) \
// GDALCheckVersion(GDAL_VERSION_MAJOR, GDAL_VERSION_MINOR, pszCallingComponentName)
//#endif

struct  GDALRPCInfo { 
    double      dfLINE_OFF;
    double      dfSAMP_OFF;
    double      dfLAT_OFF;
    double      dfLONG_OFF;
    double      dfHEIGHT_OFF;

    double      dfLINE_SCALE;
    double      dfSAMP_SCALE;
    double      dfLAT_SCALE;
    double      dfLONG_SCALE;
    double      dfHEIGHT_SCALE;

    double[20]  adfLINE_NUM_COEFF;
    double[20]  adfLINE_DEN_COEFF;
    double[20]  adfSAMP_NUM_COEFF;
    double[20]  adfSAMP_DEN_COEFF;
    
    double	dfMIN_LONG;
    double      dfMIN_LAT;
    double      dfMAX_LONG;
    double	dfMAX_LAT;
}

extern(C) int GDALExtractRPCInfo( char**, GDALRPCInfo* );

/*      Color tables.                                                   */

struct GDALColorEntry
{
    short      c1;  /*! gray, red, cyan or hue */
    short      c2;  /*! green, magenta, or lightness */      
    short      c3;  /*! blue, yellow, or saturation */
    short      c4;  /*! alpha or blackband */      
}

extern(C) GDALColorTableH 	GDALCreateColorTable( GDALPaletteInterp );
extern(C) void 			GDALDestroyColorTable( GDALColorTableH );
extern(C) GDALColorTableH 	GDALCloneColorTable( GDALColorTableH );
extern(C) GDALPaletteInterp   	GDALGetPaletteInterpretation( GDALColorTableH );
extern(C) int   		GDALGetColorEntryCount( GDALColorTableH );
extern(C) const(GDALColorEntry)* GDALGetColorEntry( GDALColorTableH, int );
extern(C) int   		GDALGetColorEntryAsRGB( GDALColorTableH, int, GDALColorEntry *);
extern(C) void   		GDALSetColorEntry( GDALColorTableH, int, const GDALColorEntry * );

extern(C) void   
GDALCreateColorRamp( GDALColorTableH hTable, 
		     int nStartIndex, const(GDALColorEntry)* psStartColor,
		     int nEndIndex, const(GDALColorEntry)*   psEndColor );

/* ==================================================================== */
/*      Raster Attribute Table						*/
/* ==================================================================== */

/** Field type of raster attribute table */
enum GDALRATFieldType {
    GFT_Integer , 
    GFT_Real,  /*double*/
    GFT_String
} 

/** Field usage of raster attribute table */
enum GDALRATFieldUsage {
    GFU_Generic = 0,  
    GFU_PixelCount = 1,
    GFU_Name = 2,
    GFU_Min = 3,
    GFU_Max = 4,
    GFU_MinMax = 5,
    GFU_Red = 6,
    GFU_Green = 7,
    GFU_Blue = 8,
    GFU_Alpha = 9,
    GFU_RedMin = 10,
    GFU_GreenMin = 11,
    GFU_BlueMin = 12,
    GFU_AlphaMin = 13,
    GFU_RedMax = 14,
    GFU_GreenMax = 15,
    GFU_BlueMax = 16,
    GFU_AlphaMax = 17,
    GFU_MaxCount
}

extern(C) GDALRasterAttributeTableH GDALCreateRasterAttributeTable();
extern(C) void GDALDestroyRasterAttributeTable( GDALRasterAttributeTableH );
extern(C) int  GDALRATGetColumnCount( GDALRasterAttributeTableH );

extern(C) const(char)*  GDALRATGetNameOfCol( GDALRasterAttributeTableH, int );

extern(C) GDALRATFieldUsage
GDALRATGetUsageOfCol( GDALRasterAttributeTableH, int );

extern(C) GDALRATFieldType 
GDALRATGetTypeOfCol( GDALRasterAttributeTableH, int );

extern(C) int GDALRATGetColOfUsage( GDALRasterAttributeTableH, 
                                    GDALRATFieldUsage );
                                    
extern(C) int GDALRATGetRowCount( GDALRasterAttributeTableH );

extern(C) const(char)* 
GDALRATGetValueAsString( GDALRasterAttributeTableH, int ,int);

extern(C) int GDALRATGetValueAsInt( GDALRasterAttributeTableH, int ,int);
extern(C) double GDALRATGetValueAsDouble( GDALRasterAttributeTableH, int ,int);

extern(C) void 
GDALRATSetValueAsString( GDALRasterAttributeTableH, int, int, const(char)* );

extern(C) void 
GDALRATSetValueAsInt( GDALRasterAttributeTableH, int, int, int );

extern(C) void 
GDALRATSetValueAsDouble( GDALRasterAttributeTableH, int, int, double );

extern(C) int 
GDALRATChangesAreWrittenToFile( GDALRasterAttributeTableH hRAT );

extern(C) CPLErr
GDALRATValuesIOAsDouble( GDALRasterAttributeTableH hRAT, GDALRWFlag eRWFlag, 
                                int iField, int iStartRow, int iLength, 
                                double* pdfData );

extern(C) CPLErr 
GDALRATValuesIOAsInteger( GDALRasterAttributeTableH hRAT, GDALRWFlag eRWFlag, 
                          int iField, int iStartRow, int iLength, int* pnData);

extern(C) CPLErr
GDALRATValuesIOAsString( GDALRasterAttributeTableH hRAT, GDALRWFlag eRWFlag, 
                         int iField, int iStartRow, int iLength, 
                         char** papszStrList);

extern(C) void GDALRATSetRowCount( GDALRasterAttributeTableH, int );

extern(C) CPLErr
GDALRATCreateColumn( GDALRasterAttributeTableH, 
                     const(char)*,GDALRATFieldType, GDALRATFieldUsage );

extern(C) CPLErr
GDALRATSetLinearBinning( GDALRasterAttributeTableH, double, double );

extern(C) int GDALRATGetLinearBinning( GDALRasterAttributeTableH, 
                                       double*, double* );
                                       
extern(C) CPLErr
GDALRATInitializeFromColorTable( GDALRasterAttributeTableH, GDALColorTableH );

extern(C) GDALColorTableH 
GDALRATTranslateToColorTable( GDALRasterAttributeTableH, int nEntryCount );

extern(C) void GDALRATDumpReadable( GDALRasterAttributeTableH, 
                                    FILE * );
                                    
extern(C) GDALRasterAttributeTableH GDALRATClone( GDALRasterAttributeTableH );

extern(C) int GDALRATGetRowOfValue( GDALRasterAttributeTableH , double );


/* ==================================================================== */
/*      GDAL Cache Management                                           */
/* ==================================================================== */

extern(C) void    GDALSetCacheMax( int nBytes );
extern(C) int     GDALGetCacheMax();
extern(C) int     GDALGetCacheUsed();
extern(C) void    GDALSetCacheMax64( GIntBig nBytes );
extern(C) GIntBig GDALGetCacheMax64();
extern(C) GIntBig GDALGetCacheUsed64();

extern(C) int     GDALFlushCacheBlock();

/* ==================================================================== */
/*      GDAL virtual memory                                             */
/* ==================================================================== */

extern(C) struct CPLVirtualMem;

extern(C) CPLVirtualMem* 
GDALDatasetGetVirtualMem( GDALDatasetH hDS, GDALRWFlag eRWFlag,
                          int nXOff, int nYOff, int nXSize, int nYSize,
                          int nBufXSize, int nBufYSize,
                          GDALDataType eBufType,
                          int nBandCount, int* panBandMap,
                          int nPixelSpace,
                          GIntBig nLineSpace,
                          GIntBig nBandSpace,
                          size_t nCacheSize,
                          size_t nPageSizeHint,
                          int bSingleThreadUsage,
                          char **papszOptions );

extern(C) CPLVirtualMem* 
GDALRasterBandGetVirtualMem( GDALRasterBandH hBand,
                             GDALRWFlag eRWFlag,
                             int nXOff, int nYOff,
                             int nXSize, int nYSize,
                             int nBufXSize, int nBufYSize,
                             GDALDataType eBufType,
                             int nPixelSpace,
                             GIntBig nLineSpace,
                             size_t nCacheSize,
                             size_t nPageSizeHint,
                             int bSingleThreadUsage,
                             char **papszOptions );

extern(C) CPLVirtualMem* 
GDALGetVirtualMemAuto( GDALRasterBandH hBand,
                       GDALRWFlag eRWFlag,
                       int* pnPixelSpace,
                       GIntBig *pnLineSpace,
                       char** papszOptions );

enum GDALTileOrganization
{
    GTO_TIP,
    GTO_BIT,
    GTO_BSQ
} 

extern(C) CPLVirtualMem* 
GDALDatasetGetTiledVirtualMem( GDALDatasetH hDS,
                               GDALRWFlag eRWFlag,
                               int nXOff, int nYOff,
                               int nXSize, int nYSize,
                               int nTileXSize, int nTileYSize,
                               GDALDataType eBufType,
                               int nBandCount, int* panBandMap,
                               GDALTileOrganization eTileOrganization,
                               size_t nCacheSize,
                               int bSingleThreadUsage,
                               char **papszOptions );
                               
extern(C) CPLVirtualMem* 
GDALRasterBandGetTiledVirtualMem( GDALRasterBandH hBand,
                                  GDALRWFlag eRWFlag,
                                  int nXOff, int nYOff,
                                  int nXSize, int nYSize,
                                  int nTileXSize, int nTileYSize,
                                  GDALDataType eBufType,
                                  size_t nCacheSize,
                                  int bSingleThreadUsage,
                                  char** papszOptions );
                                  
extern(C) CPLXMLNode* 
GDALGetJPEG2000Structure(const(char)* pszFilename,
                         char** papszOptions);
                         
                         
/* ==================================================================== */
/*      OGR API (ogr_api.h )                                            */
/* ==================================================================== */

debug 
{
struct OGRGeometryHS;
alias OGRGeometryH = OGRGeometryHS*;
}
else 
{
alias OGRGeometryH = void*;
}

debug
{
  struct OGRSpatialReferenceHS;
  alias OGRSpatialReferenceH = OGRSpatialReferenceHS*;
  struct OGRCoordinateTransformationHS;
  alias OGRCoordinateTransformationH = OGRCoordinateTransformationHS*;
}
else 
{
  alias OGRSpatialReferenceH = void *;                            
  alias OGRCoordinateTransformationH = void*;
}

//TODO;  Figure out what to do with:
// struct _CPLXMLNode;

/* From base OGRGeometry class */

extern(C) OGRErr OGR_G_CreateFromWkb( ubyte*, OGRSpatialReferenceH, 
                                      OGRGeometryH*, int );
                     
extern(C) OGRErr OGR_G_CreateFromWkt( char**, OGRSpatialReferenceH, 
                                    OGRGeometryH * );
extern(C) OGRErr OGR_G_CreateFromFgf( ubyte*, OGRSpatialReferenceH, 
                                    OGRGeometryH *, int, int* );
extern(C) void OGR_G_DestroyGeometry( OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_CreateGeometry( OGRwkbGeometryType );
extern(C) OGRGeometryH OGR_G_ApproximateArcAngles( 
    double dfCenterX, double dfCenterY, double dfZ,
    double dfPrimaryRadius, double dfSecondaryAxis, double dfRotation, 
    double dfStartAngle, double dfEndAngle,
    double dfMaxAngleStepSizeDegrees );

extern(C) OGRGeometryH OGR_G_ForceToPolygon( OGRGeometryH );
extern(C) OGRGeometryH OGR_G_ForceToLineString( OGRGeometryH );
extern(C) OGRGeometryH OGR_G_ForceToMultiPolygon( OGRGeometryH );
extern(C) OGRGeometryH OGR_G_ForceToMultiPoint( OGRGeometryH );
extern(C) OGRGeometryH OGR_G_ForceToMultiLineString( OGRGeometryH );
extern(C) OGRGeometryH OGR_G_ForceTo( OGRGeometryH hGeom,
                                    OGRwkbGeometryType eTargetType,
                                    char** papszOptions );

extern(C) int    OGR_G_GetDimension( OGRGeometryH );
extern(C) int    OGR_G_GetCoordinateDimension( OGRGeometryH );
extern(C) void   OGR_G_SetCoordinateDimension( OGRGeometryH, int );
extern(C) OGRGeometryH OGR_G_Clone( OGRGeometryH );
extern(C) void   OGR_G_GetEnvelope( OGRGeometryH, OGREnvelope * );
extern(C) void   OGR_G_GetEnvelope3D( OGRGeometryH, OGREnvelope3D * );
extern(C) OGRErr OGR_G_ImportFromWkb( OGRGeometryH, ubyte*, int );
extern(C) OGRErr OGR_G_ExportToWkb( OGRGeometryH, OGRwkbByteOrder, ubyte*);
extern(C) OGRErr OGR_G_ExportToIsoWkb( OGRGeometryH, OGRwkbByteOrder, ubyte*);
extern(C) int    OGR_G_WkbSize( OGRGeometryH hGeom );
extern(C) OGRErr OGR_G_ImportFromWkt( OGRGeometryH, char** );
extern(C) OGRErr OGR_G_ExportToWkt( OGRGeometryH, char** );
extern(C) OGRErr OGR_G_ExportToIsoWkt( OGRGeometryH, char** );
extern(C) OGRwkbGeometryType  OGR_G_GetGeometryType( OGRGeometryH );
extern(C) const(char)* OGR_G_GetGeometryName( OGRGeometryH );

extern(C) void   OGR_G_DumpReadable( OGRGeometryH, FILE *, const(char)* );
extern(C) void   OGR_G_FlattenTo2D( OGRGeometryH );
extern(C) void   OGR_G_CloseRings( OGRGeometryH );

extern(C) OGRGeometryH OGR_G_CreateFromGML( const(char)* );
extern(C) char* OGR_G_ExportToGML( OGRGeometryH );
extern(C) char* OGR_G_ExportToGMLEx( OGRGeometryH, char** papszOptions );

extern(C) OGRGeometryH OGR_G_CreateFromGMLTree( const(CPLXMLNode)* );
extern(C) CPLXMLNode* OGR_G_ExportToGMLTree( OGRGeometryH );
extern(C) CPLXMLNode* OGR_G_ExportEnvelopeToGMLTree( OGRGeometryH );

extern(C) char* OGR_G_ExportToKML( OGRGeometryH, const(char)* pszAltitudeMode );

extern(C) char* OGR_G_ExportToJson( OGRGeometryH );
extern(C) char* OGR_G_ExportToJsonEx( OGRGeometryH, char** papszOptions );
extern(C) OGRGeometryH  OGR_G_CreateGeometryFromJson( const(char)* );

extern(C) void OGR_G_AssignSpatialReference( OGRGeometryH, 
                                       OGRSpatialReferenceH );
                                    
                                       
                                       
extern(C) OGRSpatialReferenceH  OGR_G_GetSpatialReference( OGRGeometryH );
extern(C) OGRErr  OGR_G_Transform( OGRGeometryH, OGRCoordinateTransformationH );
extern(C) OGRErr  OGR_G_TransformTo( OGRGeometryH, OGRSpatialReferenceH );

extern(C) OGRGeometryH  OGR_G_Simplify( OGRGeometryH hThis, double tolerance );
extern(C) OGRGeometryH  OGR_G_SimplifyPreserveTopology( OGRGeometryH hThis, double tolerance );

extern(C) void    OGR_G_Segmentize(OGRGeometryH hGeom, double dfMaxLength );
extern(C) int     OGR_G_Intersects( OGRGeometryH, OGRGeometryH );
extern(C) int     OGR_G_Equals( OGRGeometryH, OGRGeometryH );
/*int     OGR_G_EqualsExact( OGRGeometryH, OGRGeometryH, double );*/
extern(C) int     OGR_G_Disjoint( OGRGeometryH, OGRGeometryH );
extern(C) int     OGR_G_Touches( OGRGeometryH, OGRGeometryH );
extern(C) int     OGR_G_Crosses( OGRGeometryH, OGRGeometryH );
extern(C) int     OGR_G_Within( OGRGeometryH, OGRGeometryH );
extern(C) int     OGR_G_Contains( OGRGeometryH, OGRGeometryH );
extern(C) int     OGR_G_Overlaps( OGRGeometryH, OGRGeometryH );

extern(C) OGRGeometryH  OGR_G_Boundary( OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_ConvexHull( OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_Buffer( OGRGeometryH, double, int );
extern(C) OGRGeometryH  OGR_G_Intersection( OGRGeometryH, OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_Union( OGRGeometryH, OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_UnionCascaded( OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_PointOnSurface( OGRGeometryH );
/*OGRGeometryH  OGR_G_Polygonize( OGRGeometryH *, int);*/
/*OGRGeometryH  OGR_G_Polygonizer_getCutEdges( OGRGeometryH *, int);*/
/*OGRGeometryH  OGR_G_LineMerge( OGRGeometryH );*/

extern(C) OGRGeometryH  OGR_G_Difference( OGRGeometryH, OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_SymDifference( OGRGeometryH, OGRGeometryH );
extern(C) double  OGR_G_Distance( OGRGeometryH, OGRGeometryH );
extern(C) double  OGR_G_Length( OGRGeometryH );
extern(C) double  OGR_G_Area( OGRGeometryH );
extern(C) int     OGR_G_Centroid( OGRGeometryH, OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_Value( OGRGeometryH, double dfDistance );

extern(C) void    OGR_G_Empty( OGRGeometryH );
extern(C) int     OGR_G_IsEmpty( OGRGeometryH );
extern(C) int     OGR_G_IsValid( OGRGeometryH );
/*char     *OGR_G_IsValidReason( OGRGeometryH );*/
extern(C) int     OGR_G_IsSimple( OGRGeometryH );
extern(C) int     OGR_G_IsRing( OGRGeometryH );
 
extern(C) OGRGeometryH  OGR_G_Polygonize( OGRGeometryH );

/* backward compatibility (non-standard methods) */
deprecated("Non standard method. Use OGR_G_Intersects() instead")
{
    extern(C) int     OGR_G_Intersect( OGRGeometryH, OGRGeometryH );
}

deprecated("Non standard method. Use OGR_G_Equals() instead")
{
     extern(C) int     OGR_G_Equal( OGRGeometryH, OGRGeometryH );
}

deprecated("Non standard method. Use OGR_G_SymDifference() instead")
{
    extern(C) OGRGeometryH  OGR_G_SymmetricDifference( OGRGeometryH, OGRGeometryH );
}

deprecated("Non standard method. Use OGR_G_Area() instead")
{
    extern(C) double  OGR_G_GetArea( OGRGeometryH );
}

deprecated("Non standard method. Use OGR_G_Boundary() instead")
{
    extern(C) OGRGeometryH  OGR_G_GetBoundary( OGRGeometryH );
}

/* Methods for getting/setting vertices in points, line strings and rings */
extern(C) int     OGR_G_GetPointCount( OGRGeometryH );
extern(C) int     OGR_G_GetPoints( OGRGeometryH hGeom,
                                void* pabyX, int nXStride,
                                void* pabyY, int nYStride,
                                void* pabyZ, int nZStride);
extern(C) double  OGR_G_GetX( OGRGeometryH, int );
extern(C) double  OGR_G_GetY( OGRGeometryH, int );
extern(C) double  OGR_G_GetZ( OGRGeometryH, int );
extern(C) void    OGR_G_GetPoint( OGRGeometryH, int iPoint, 
                               double*, double*, double* );
extern(C) void    OGR_G_SetPointCount( OGRGeometryH hGeom, int nNewPointCount );
extern(C) void    OGR_G_SetPoint( OGRGeometryH, int iPoint, 
                               double, double, double );
extern(C) void    OGR_G_SetPoint_2D( OGRGeometryH, int iPoint, 
                                  double, double );
extern(C) void    OGR_G_AddPoint( OGRGeometryH, double, double, double );
extern(C) void    OGR_G_AddPoint_2D( OGRGeometryH, double, double );
extern(C) void    OGR_G_SetPoints( OGRGeometryH hGeom, int nPointsIn,
                                void* pabyX, int nXStride,
                                void* pabyY, int nYStride,
                                void* pabyZ, int nZStride );

/* Methods for getting/setting rings and members collections */

extern(C) int     OGR_G_GetGeometryCount( OGRGeometryH );
extern(C) OGRGeometryH  OGR_G_GetGeometryRef( OGRGeometryH, int );
extern(C) OGRErr  OGR_G_AddGeometry( OGRGeometryH, OGRGeometryH );
extern(C) OGRErr  OGR_G_AddGeometryDirectly( OGRGeometryH, OGRGeometryH );
extern(C) OGRErr  OGR_G_RemoveGeometry( OGRGeometryH, int, int );
  

extern(C) int  OGR_G_HasCurveGeometry( OGRGeometryH, int bLookForNonLinear );
extern(C) OGRGeometryH  OGR_G_GetLinearGeometry( OGRGeometryH hGeom,
                                              double dfMaxAngleStepSizeDegrees,
                                              char** papszOptions);
extern(C) OGRGeometryH  OGR_G_GetCurveGeometry( OGRGeometryH hGeom,
                                             char** papszOptions );

extern(C) OGRGeometryH  OGRBuildPolygonFromEdges( OGRGeometryH hLinesAsCollection,
                                       int bBestEffort, 
                                       int bAutoClose, 
                                       double dfTolerance,
                                       OGRErr * peErr );

extern(C) OGRErr  OGRSetGenerate_DB2_V72_BYTE_ORDER( 
    int bGenerate_DB2_V72_BYTE_ORDER );

extern(C) int  OGRGetGenerate_DB2_V72_BYTE_ORDER();

extern(C) void  OGRSetNonLinearGeometriesEnabledFlag(int bFlag);
extern(C) int  OGRGetNonLinearGeometriesEnabledFlag();

/* -------------------------------------------------------------------- */
/*      Feature related (ogr_feature.h)                                 */
/* -------------------------------------------------------------------- */

debug 
{
  struct OGRFieldDefnHS;
  struct OGRFeatureDefnHS;
  struct OGRFeatureHS;
  struct OGRStyleTableHS;
  alias OGRFieldDefnH   = OGRFieldDefnHS*;
  alias OGRFeatureDefnH = OGRFeatureDefnHS*;
  alias OGRFeatureH     = OGRFeatureHS*;
  alias OGRStyleTableH  = OGRStyleTableHS*;
}
else 
{
  alias OGRFieldDefnH   = void*;
  alias OGRFeatureDefnH = void*;
  alias OGRFeatureH     = void*;
  alias OGRStyleTableH  = void*;
}

struct OGRGeomFieldDefnHS;
alias OGRGeomFieldDefnH  = OGRGeomFieldDefnHS*;

/* OGRFieldDefn */

extern(C) OGRFieldDefnH  OGR_Fld_Create( const(char)*, OGRFieldType ); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) void    OGR_Fld_Destroy( OGRFieldDefnH );

extern(C) void    OGR_Fld_SetName( OGRFieldDefnH, const(char)* );
extern(C) const(char)* OGR_Fld_GetNameRef( OGRFieldDefnH );
extern(C) OGRFieldType  OGR_Fld_GetType( OGRFieldDefnH );
extern(C) void    OGR_Fld_SetType( OGRFieldDefnH, OGRFieldType );
extern(C) OGRFieldSubType  OGR_Fld_GetSubType( OGRFieldDefnH );
extern(C) void    OGR_Fld_SetSubType( OGRFieldDefnH, OGRFieldSubType );
extern(C) OGRJustification  OGR_Fld_GetJustify( OGRFieldDefnH );
extern(C) void    OGR_Fld_SetJustify( OGRFieldDefnH, OGRJustification );
extern(C) int     OGR_Fld_GetWidth( OGRFieldDefnH );
extern(C) void    OGR_Fld_SetWidth( OGRFieldDefnH, int );
extern(C) int     OGR_Fld_GetPrecision( OGRFieldDefnH );
extern(C) void    OGR_Fld_SetPrecision( OGRFieldDefnH, int );
extern(C) void    OGR_Fld_Set( OGRFieldDefnH, const(char)*, OGRFieldType, 
                               int, int, OGRJustification );
extern(C) int     OGR_Fld_IsIgnored( OGRFieldDefnH hDefn );
extern(C) void    OGR_Fld_SetIgnored( OGRFieldDefnH hDefn, int );
extern(C) int     OGR_Fld_IsNullable( OGRFieldDefnH hDefn );
extern(C) void    OGR_Fld_SetNullable( OGRFieldDefnH hDefn, int );
extern(C) const(char)* OGR_Fld_GetDefault( OGRFieldDefnH hDefn );
extern(C) void    OGR_Fld_SetDefault( OGRFieldDefnH hDefn, const(char)* );
extern(C) int     OGR_Fld_IsDefaultDriverSpecific( OGRFieldDefnH hDefn );

extern(C) const(char)* OGR_GetFieldTypeName( OGRFieldType );
extern(C) const(char)* OGR_GetFieldSubTypeName( OGRFieldSubType );
extern(C) int  OGR_AreTypeSubTypeCompatible( OGRFieldType eType,
                                             OGRFieldSubType eSubType );

/* OGRGeomFieldDefnH */

extern(C) OGRGeomFieldDefnH     OGR_GFld_Create( const(char)*, OGRwkbGeometryType ); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) void                  OGR_GFld_Destroy( OGRGeomFieldDefnH );

extern(C) void                  OGR_GFld_SetName( OGRGeomFieldDefnH, const(char)* );
extern(C) const(char)*          OGR_GFld_GetNameRef( OGRGeomFieldDefnH );

extern(C) OGRwkbGeometryType    OGR_GFld_GetType( OGRGeomFieldDefnH );
extern(C) void                  OGR_GFld_SetType( OGRGeomFieldDefnH, OGRwkbGeometryType );

extern(C) OGRSpatialReferenceH  OGR_GFld_GetSpatialRef( OGRGeomFieldDefnH );
extern(C) void                  OGR_GFld_SetSpatialRef( OGRGeomFieldDefnH,
                                                        OGRSpatialReferenceH hSRS );

extern(C) int                   OGR_GFld_IsNullable( OGRGeomFieldDefnH hDefn );
extern(C) void                  OGR_GFld_SetNullable( OGRGeomFieldDefnH hDefn, int );

extern(C) int                   OGR_GFld_IsIgnored( OGRGeomFieldDefnH hDefn );
extern(C) void                  OGR_GFld_SetIgnored( OGRGeomFieldDefnH hDefn, int );

/* OGRFeatureDefn */

extern(C) OGRFeatureDefnH  OGR_FD_Create( const(char)* ); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) void    OGR_FD_Destroy( OGRFeatureDefnH );
extern(C) void    OGR_FD_Release( OGRFeatureDefnH );
extern(C) const(char)*  OGR_FD_GetName( OGRFeatureDefnH );
extern(C) int     OGR_FD_GetFieldCount( OGRFeatureDefnH );
extern(C) OGRFieldDefnH  OGR_FD_GetFieldDefn( OGRFeatureDefnH, int );
extern(C) int     OGR_FD_GetFieldIndex( OGRFeatureDefnH, const(char)* );
extern(C) void    OGR_FD_AddFieldDefn( OGRFeatureDefnH, OGRFieldDefnH );
extern(C) OGRErr  OGR_FD_DeleteFieldDefn( OGRFeatureDefnH hDefn, int iField );
extern(C) OGRErr  OGR_FD_ReorderFieldDefns( OGRFeatureDefnH hDefn, int* panMap );
extern(C) OGRwkbGeometryType  OGR_FD_GetGeomType( OGRFeatureDefnH );
extern(C) void    OGR_FD_SetGeomType( OGRFeatureDefnH, OGRwkbGeometryType );
extern(C) int     OGR_FD_IsGeometryIgnored( OGRFeatureDefnH );
extern(C) void    OGR_FD_SetGeometryIgnored( OGRFeatureDefnH, int );
extern(C) int     OGR_FD_IsStyleIgnored( OGRFeatureDefnH );
extern(C) void    OGR_FD_SetStyleIgnored( OGRFeatureDefnH, int );
extern(C) int     OGR_FD_Reference( OGRFeatureDefnH );
extern(C) int     OGR_FD_Dereference( OGRFeatureDefnH );
extern(C) int     OGR_FD_GetReferenceCount( OGRFeatureDefnH );

extern(C) int                OGR_FD_GetGeomFieldCount( OGRFeatureDefnH hFDefn );
extern(C) OGRGeomFieldDefnH  OGR_FD_GetGeomFieldDefn(  OGRFeatureDefnH hFDefn,
                                                       int i );
extern(C) int                OGR_FD_GetGeomFieldIndex( OGRFeatureDefnH hFDefn,
                                                       const(char)*pszName);

extern(C) void               OGR_FD_AddGeomFieldDefn( OGRFeatureDefnH hFDefn,
                                                      OGRGeomFieldDefnH hGFldDefn);
extern(C) OGRErr             OGR_FD_DeleteGeomFieldDefn( OGRFeatureDefnH hFDefn,
                                                         int iGeomField );
extern(C) int                OGR_FD_IsSame( OGRFeatureDefnH hFDefn,
                                            OGRFeatureDefnH hOtherFDefn );
/* OGRFeature */

extern(C) OGRFeatureH  OGR_F_Create( OGRFeatureDefnH ); //TODO ? CPL_WARN_UNUSED_RESULT;
extern(C) void    OGR_F_Destroy( OGRFeatureH );
extern(C) OGRFeatureDefnH  OGR_F_GetDefnRef( OGRFeatureH );

extern(C) OGRErr  OGR_F_SetGeometryDirectly( OGRFeatureH, OGRGeometryH );
extern(C) OGRErr  OGR_F_SetGeometry( OGRFeatureH, OGRGeometryH );
extern(C) OGRGeometryH  OGR_F_GetGeometryRef( OGRFeatureH );
extern(C) OGRGeometryH  OGR_F_StealGeometry( OGRFeatureH );
extern(C) OGRFeatureH  OGR_F_Clone( OGRFeatureH );
extern(C) int     OGR_F_Equal( OGRFeatureH, OGRFeatureH );

extern(C) int            OGR_F_GetFieldCount( OGRFeatureH );
extern(C) OGRFieldDefnH  OGR_F_GetFieldDefnRef( OGRFeatureH, int );
extern(C) int            OGR_F_GetFieldIndex( OGRFeatureH, const(char)* );

extern(C) int       OGR_F_IsFieldSet( OGRFeatureH, int );
extern(C) void      OGR_F_UnsetField( OGRFeatureH, int );
extern(C) OGRField* OGR_F_GetRawFieldRef( OGRFeatureH, int );

extern(C) int             OGR_F_GetFieldAsInteger( OGRFeatureH, int );
extern(C) GIntBig         OGR_F_GetFieldAsInteger64( OGRFeatureH, int );
extern(C) double          OGR_F_GetFieldAsDouble( OGRFeatureH, int );
extern(C) const(char)*    OGR_F_GetFieldAsString( OGRFeatureH, int );
extern(C) const(int)*     OGR_F_GetFieldAsIntegerList( OGRFeatureH, int, int* );
extern(C) const(GIntBig)* OGR_F_GetFieldAsInteger64List( OGRFeatureH, int, int* );
extern(C) const(double)*  OGR_F_GetFieldAsDoubleList( OGRFeatureH, int, int* );
extern(C) char**          OGR_F_GetFieldAsStringList( OGRFeatureH, int );
extern(C) GByte*          OGR_F_GetFieldAsBinary( OGRFeatureH, int, int * );
extern(C) int             OGR_F_GetFieldAsDateTime( OGRFeatureH, int, int*, int*, int*,
                                                    int*, int*, int*, int* );
extern(C) int             OGR_F_GetFieldAsDateTimeEx( OGRFeatureH hFeat, int iField,
                                int* pnYear, int* pnMonth, int*  pnDay,
                                int* pnHour, int* pnMinute, float* pfSecond,
                                int* pnTZFlag );

extern(C) void    OGR_F_SetFieldInteger( OGRFeatureH, int, int );
extern(C) void    OGR_F_SetFieldInteger64( OGRFeatureH, int, GIntBig );
extern(C) void    OGR_F_SetFieldDouble( OGRFeatureH, int, double );
extern(C) void    OGR_F_SetFieldString( OGRFeatureH, int, const(char)* );
extern(C) void    OGR_F_SetFieldIntegerList( OGRFeatureH, int, int, int* );
extern(C) void    OGR_F_SetFieldInteger64List( OGRFeatureH, int, int, const(GIntBig)* );
extern(C) void    OGR_F_SetFieldDoubleList( OGRFeatureH, int, int, double* );
extern(C) void    OGR_F_SetFieldStringList( OGRFeatureH, int, char** );
extern(C) void    OGR_F_SetFieldRaw( OGRFeatureH, int, OGRField* );
extern(C) void    OGR_F_SetFieldBinary( OGRFeatureH, int, int, GByte* );
extern(C) void    OGR_F_SetFieldDateTime( OGRFeatureH, int, 
                                       int, int, int, int, int, int, int );
extern(C) void    OGR_F_SetFieldDateTimeEx( OGRFeatureH, int, 
                                       int, int, int, int, int, float, int );

extern(C) int                OGR_F_GetGeomFieldCount( OGRFeatureH hFeat );
extern(C) OGRGeomFieldDefnH  OGR_F_GetGeomFieldDefnRef( OGRFeatureH hFeat,
                                                     int iField );
extern(C) int                OGR_F_GetGeomFieldIndex( OGRFeatureH hFeat,
                                                      const(char)*pszName);

extern(C) OGRGeometryH       OGR_F_GetGeomFieldRef( OGRFeatureH hFeat,
                                                    int iField );
extern(C) OGRErr             OGR_F_SetGeomFieldDirectly( OGRFeatureH hFeat,
                                                      int iField,
                                                      OGRGeometryH hGeom );
extern(C) OGRErr             OGR_F_SetGeomField( OGRFeatureH hFeat,
                                                 int iField, OGRGeometryH hGeom );

extern(C) GIntBig  OGR_F_GetFID( OGRFeatureH );
extern(C) OGRErr   OGR_F_SetFID( OGRFeatureH, GIntBig );
extern(C) void     OGR_F_DumpReadable( OGRFeatureH, FILE* );
extern(C) OGRErr   OGR_F_SetFrom( OGRFeatureH, OGRFeatureH, int );
extern(C) OGRErr   OGR_F_SetFromWithMap( OGRFeatureH, OGRFeatureH, int , int* );

extern(C) const(char)*    OGR_F_GetStyleString( OGRFeatureH );
extern(C) void            OGR_F_SetStyleString( OGRFeatureH, const(char)* );
extern(C) void            OGR_F_SetStyleStringDirectly( OGRFeatureH, char* );
extern(C) OGRStyleTableH  OGR_F_GetStyleTable( OGRFeatureH );
extern(C) void            OGR_F_SetStyleTableDirectly( OGRFeatureH, 
                                                       OGRStyleTableH );
extern(C) void            OGR_F_SetStyleTable( OGRFeatureH, OGRStyleTableH );

extern(C) void            OGR_F_FillUnsetWithDefault( OGRFeatureH hFeat,
                                                      int bNotNullableOnly,
                                                      char** papszOptions );
extern(C) int             OGR_F_Validate( OGRFeatureH, int nValidateFlags, 
                                          int bEmitError );

/* -------------------------------------------------------------------- */
/*      ogrsf_frmts.h                                                   */
/* -------------------------------------------------------------------- */

debug 
{
  struct OGRLayerHS;
  alias OGRLayerH = OGRLayerHS*;
  struct OGRDataSourceHS;
  alias OGRDataSourceH = OGRDataSourceHS*;
  struct OGRDriverHS;
  alias OGRSFDriverH = OGRDriverHS*;
}
else 
{
  alias OGRLayerH = void*;
  alias OGRDataSourceH = void*;
  alias OGRSFDriverH = void*;
}

/* OGRLayer */

extern(C) const(char)* OGR_L_GetName( OGRLayerH );
extern(C) OGRwkbGeometryType  OGR_L_GetGeomType( OGRLayerH );
extern(C) OGRGeometryH  OGR_L_GetSpatialFilter( OGRLayerH );
extern(C) void    OGR_L_SetSpatialFilter( OGRLayerH, OGRGeometryH );
extern(C) void    OGR_L_SetSpatialFilterRect( OGRLayerH, 
                                           double, double, double, double );
extern(C) void    OGR_L_SetSpatialFilterEx( OGRLayerH, int iGeomField,
                                           OGRGeometryH hGeom );
extern(C) void    OGR_L_SetSpatialFilterRectEx( OGRLayerH, int iGeomField,
                                               double dfMinX, double dfMinY,
                                               double dfMaxX, double dfMaxY );
extern(C) OGRErr       OGR_L_SetAttributeFilter( OGRLayerH, const(char)* );
extern(C) void         OGR_L_ResetReading( OGRLayerH );
extern(C) OGRFeatureH  OGR_L_GetNextFeature( OGRLayerH );
extern(C) OGRErr       OGR_L_SetNextByIndex( OGRLayerH, GIntBig );
extern(C) OGRFeatureH  OGR_L_GetFeature( OGRLayerH, GIntBig );
extern(C) OGRErr       OGR_L_SetFeature( OGRLayerH, OGRFeatureH );
extern(C) OGRErr       OGR_L_CreateFeature( OGRLayerH, OGRFeatureH );
extern(C) OGRErr       OGR_L_DeleteFeature( OGRLayerH, GIntBig );
extern(C) OGRFeatureDefnH  OGR_L_GetLayerDefn( OGRLayerH );
extern(C) OGRSpatialReferenceH  OGR_L_GetSpatialRef( OGRLayerH );
extern(C) int          OGR_L_FindFieldIndex( OGRLayerH, const(char)*, int bExactMatch );
extern(C) GIntBig      OGR_L_GetFeatureCount( OGRLayerH, int );
extern(C) OGRErr       OGR_L_GetExtent( OGRLayerH, OGREnvelope *, int );
extern(C) OGRErr       OGR_L_GetExtentEx( OGRLayerH, int iGeomField,
                                          OGREnvelope *psExtent, int bForce );
extern(C) int     OGR_L_TestCapability( OGRLayerH, const(char)* );
extern(C) OGRErr  OGR_L_CreateField( OGRLayerH, OGRFieldDefnH, int );
extern(C) OGRErr  OGR_L_CreateGeomField( OGRLayerH hLayer,
                                      OGRGeomFieldDefnH hFieldDefn, int bForce );
extern(C) OGRErr  OGR_L_DeleteField( OGRLayerH, int iField );
extern(C) OGRErr  OGR_L_ReorderFields( OGRLayerH, int* panMap );
extern(C) OGRErr  OGR_L_ReorderField( OGRLayerH, int iOldFieldPos, int iNewFieldPos );
extern(C) OGRErr  OGR_L_AlterFieldDefn( OGRLayerH, int iField, OGRFieldDefnH hNewFieldDefn, int nFlags );
extern(C) OGRErr  OGR_L_StartTransaction( OGRLayerH );
extern(C) OGRErr  OGR_L_CommitTransaction( OGRLayerH );
extern(C) OGRErr  OGR_L_RollbackTransaction( OGRLayerH );
extern(C) int     OGR_L_Reference( OGRLayerH );
extern(C) int     OGR_L_Dereference( OGRLayerH );
extern(C) int     OGR_L_GetRefCount( OGRLayerH );
extern(C) OGRErr  OGR_L_SyncToDisk( OGRLayerH );
extern(C) GIntBig  OGR_L_GetFeaturesRead( OGRLayerH );
extern(C) const(char)* OGR_L_GetFIDColumn( OGRLayerH );
extern(C) const(char)*  OGR_L_GetGeometryColumn( OGRLayerH );
extern(C) OGRStyleTableH  OGR_L_GetStyleTable( OGRLayerH );
extern(C) void    OGR_L_SetStyleTableDirectly( OGRLayerH, OGRStyleTableH );
extern(C) void    OGR_L_SetStyleTable( OGRLayerH, OGRStyleTableH );
extern(C) OGRErr  OGR_L_SetIgnoredFields( OGRLayerH, const(char)** );
extern(C) OGRErr  OGR_L_Intersection( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );
extern(C) OGRErr  OGR_L_Union( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );
extern(C) OGRErr  OGR_L_SymDifference( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );
extern(C) OGRErr  OGR_L_Identity( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );
extern(C) OGRErr  OGR_L_Update( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );
extern(C) OGRErr  OGR_L_Clip( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );
extern(C) OGRErr  OGR_L_Erase( OGRLayerH, OGRLayerH, OGRLayerH, char**, GDALProgressFunc, void * );

/* OGRDataSource */

extern(C) void    OGR_DS_Destroy( OGRDataSourceH );
extern(C) const(char)* OGR_DS_GetName( OGRDataSourceH );
extern(C) int     OGR_DS_GetLayerCount( OGRDataSourceH );
extern(C) OGRLayerH  OGR_DS_GetLayer( OGRDataSourceH, int );
extern(C) OGRLayerH  OGR_DS_GetLayerByName( OGRDataSourceH, const(char)* );
extern(C) OGRErr     OGR_DS_DeleteLayer( OGRDataSourceH, int );
extern(C) OGRSFDriverH  OGR_DS_GetDriver( OGRDataSourceH );
extern(C) OGRLayerH  OGR_DS_CreateLayer( OGRDataSourceH, const(char)*, 
                                      OGRSpatialReferenceH, OGRwkbGeometryType,
                                      char ** );
extern(C) OGRLayerH  OGR_DS_CopyLayer( OGRDataSourceH, OGRLayerH, const(char)*,
                                    char ** );
extern(C) int     OGR_DS_TestCapability( OGRDataSourceH, const(char)* );
extern(C) OGRLayerH  OGR_DS_ExecuteSQL( OGRDataSourceH, const(char)*,
                                     OGRGeometryH, const(char)* );
extern(C) void    OGR_DS_ReleaseResultSet( OGRDataSourceH, OGRLayerH );
extern(C) int     OGR_DS_Reference( OGRDataSourceH );
extern(C) int     OGR_DS_Dereference( OGRDataSourceH );
extern(C) int     OGR_DS_GetRefCount( OGRDataSourceH );
extern(C) int     OGR_DS_GetSummaryRefCount( OGRDataSourceH );
extern(C) OGRErr  OGR_DS_SyncToDisk( OGRDataSourceH );
extern(C) OGRStyleTableH  OGR_DS_GetStyleTable( OGRDataSourceH );
extern(C) void    OGR_DS_SetStyleTableDirectly( OGRDataSourceH, OGRStyleTableH );
extern(C) void    OGR_DS_SetStyleTable( OGRDataSourceH, OGRStyleTableH );

/* OGRSFDriver */

extern(C) const(char)*  OGR_Dr_GetName( OGRSFDriverH );
extern(C) OGRDataSourceH  OGR_Dr_Open( OGRSFDriverH, const(char)*, int ); //TODO ? CPL_WARN_UNUSED_RESULT;
extern(C) int  OGR_Dr_TestCapability( OGRSFDriverH, const(char)* );
extern(C) OGRDataSourceH  OGR_Dr_CreateDataSource( OGRSFDriverH, const(char)*,
                                                char ** ); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) OGRDataSourceH  OGR_Dr_CopyDataSource( OGRSFDriverH,  OGRDataSourceH, 
                                              const(char)*, char ** ); // TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) OGRErr  OGR_Dr_DeleteDataSource( OGRSFDriverH, const(char)* );

/* OGRSFDriverRegistrar */

extern(C) OGRDataSourceH  OGROpen( const(char)*, int, OGRSFDriverH * ); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) OGRDataSourceH  OGROpenShared( const(char)*, int, OGRSFDriverH * ); //TODO ? CPL_WARN_UNUSED_RESULT;
extern(C) OGRErr   OGRReleaseDataSource( OGRDataSourceH );
extern(C) void     OGRRegisterDriver( OGRSFDriverH );
extern(C) void     OGRDeregisterDriver( OGRSFDriverH );
extern(C) int      OGRGetDriverCount();
extern(C) OGRSFDriverH  OGRGetDriver( int );
extern(C) OGRSFDriverH  OGRGetDriverByName( const(char)* );
extern(C) int      OGRGetOpenDSCount();
extern(C) OGRDataSourceH  OGRGetOpenDS( int iDS );


/* note: this is also declared in ogrsf_frmts.h */
extern(C) void  OGRRegisterAll();
extern(C) void  OGRCleanupAll();

/* -------------------------------------------------------------------- */
/*      ogrsf_featurestyle.h                                            */
/* -------------------------------------------------------------------- */

debug 
{
struct OGRStyleMgrHS;
struct OGRStyleToolHS;
alias OGRStyleMgrH = OGRStyleMgrHS*;
alias OGRStyleToolH = OGRStyleToolHS*;
}
else
{
alias OGRStyleMgrH = void*;
alias OGRStyleToolH = void*;
}

/* OGRStyleMgr */

extern(C) OGRStyleMgrH  OGR_SM_Create(OGRStyleTableH hStyleTable); //TODO ? CPL_WARN_UNUSED_RESULT;
extern(C) void     OGR_SM_Destroy(OGRStyleMgrH hSM);

extern(C) const(char)*  OGR_SM_InitFromFeature(OGRStyleMgrH hSM, 
                                           OGRFeatureH hFeat);
extern(C) int      OGR_SM_InitStyleString(OGRStyleMgrH hSM, 
                                       const(char)*pszStyleString);
extern(C) int      OGR_SM_GetPartCount(OGRStyleMgrH hSM, 
                                    const(char)*pszStyleString);
extern(C) OGRStyleToolH  OGR_SM_GetPart(OGRStyleMgrH hSM, int nPartId, 
                                     const(char)*pszStyleString);
extern(C) int      OGR_SM_AddPart(OGRStyleMgrH hSM, OGRStyleToolH hST);
extern(C) int      OGR_SM_AddStyle(OGRStyleMgrH hSM, const(char)*pszStyleName, 
                               const(char)*pszStyleString);

/* OGRStyleTool */

extern(C) OGRStyleToolH  OGR_ST_Create(OGRSTClassId eClassId); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) void     OGR_ST_Destroy(OGRStyleToolH hST);

extern(C) OGRSTClassId  OGR_ST_GetType(OGRStyleToolH hST);

extern(C) OGRSTUnitId  OGR_ST_GetUnit(OGRStyleToolH hST);
extern(C) void     OGR_ST_SetUnit(OGRStyleToolH hST, OGRSTUnitId eUnit, 
                               double dfGroundPaperScale);

extern(C) const(char)* OGR_ST_GetParamStr(OGRStyleToolH hST, int eParam, int *bValueIsNull);
extern(C) int      OGR_ST_GetParamNum(OGRStyleToolH hST, int eParam, int *bValueIsNull);
extern(C) double   OGR_ST_GetParamDbl(OGRStyleToolH hST, int eParam, int *bValueIsNull);
extern(C) void     OGR_ST_SetParamStr(OGRStyleToolH hST, int eParam, const(char)*pszValue);
extern(C) void     OGR_ST_SetParamNum(OGRStyleToolH hST, int eParam, int nValue);
extern(C) void     OGR_ST_SetParamDbl(OGRStyleToolH hST, int eParam, double dfValue);
extern(C) const(char)* OGR_ST_GetStyleString(OGRStyleToolH hST);

extern(C) int  OGR_ST_GetRGBFromString(OGRStyleToolH hST, const(char)*pszColor, 
                                    int *pnRed, int *pnGreen, int *pnBlue, 
                                    int *pnAlpha);

/* OGRStyleTable */

extern(C) OGRStyleTableH   OGR_STBL_Create(); //TODO: ? CPL_WARN_UNUSED_RESULT;
extern(C) void     OGR_STBL_Destroy( OGRStyleTableH hSTBL ); 
extern(C) int      OGR_STBL_AddStyle( OGRStyleTableH hStyleTable,
                                   const(char)*pszName,
                                   const(char)*pszStyleString);
extern(C) int      OGR_STBL_SaveStyleTable( OGRStyleTableH hStyleTable,
                                         const(char)*pszFilename );
extern(C) int      OGR_STBL_LoadStyleTable( OGRStyleTableH hStyleTable,
                                         const(char)*pszFilename );
extern(C) const(char)* OGR_STBL_Find( OGRStyleTableH hStyleTable, const(char)*pszName );
extern(C) void     OGR_STBL_ResetStyleStringReading( OGRStyleTableH hStyleTable );
extern(C) const(char)* OGR_STBL_GetNextStyle( OGRStyleTableH hStyleTable);
extern(C) const(char)* OGR_STBL_GetLastStyleName( OGRStyleTableH hStyleTable);

/*****************************************************************************************
 * From ogr_core.h
 *****************************************************************************************/
 
/* The following definitions are from org_core.h */

extern(C) enum OGRwkbGeometryType  //From ogr_core.h
{
    wkbUnknown = 0,         /**< unknown type, non-standard */
    wkbPoint = 1,           /**< 0-dimensional geometric object, standard WKB */
    wkbLineString = 2,      /**< 1-dimensional geometric object with linear
                             *   interpolation between Points, standard WKB */
    wkbPolygon = 3,         /**< planar 2-dimensional geometric object defined
                             *   by 1 exterior boundary and 0 or more interior
                             *   boundaries, standard WKB */
    wkbMultiPoint = 4,      /**< GeometryCollection of Points, standard WKB */
    wkbMultiLineString = 5, /**< GeometryCollection of LineStrings, standard WKB */
    wkbMultiPolygon = 6,    /**< GeometryCollection of Polygons, standard WKB */
    wkbGeometryCollection = 7, /**< geometric object that is a collection of 1
                                    or more geometric objects, standard WKB */
    wkbCircularString = 8,  /**< one or more circular arc segments connected end to end,
                             *   ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCompoundCurve = 9,   /**< sequence of contiguous curves, ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCurvePolygon = 10,   /**< planar surface, defined by 1 exterior boundary
                             *   and zero or more interior boundaries, that are curves.
                             *    ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbMultiCurve = 11,     /**< GeometryCollection of Curves, ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbMultiSurface = 12,   /**< GeometryCollection of Surfaces, ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbNone = 100,          /**< non-standard, for pure attribute records */
    wkbLinearRing = 101,    /**< non-standard, just for createGeometry() */
    wkbCircularStringZ = 1008,  /**< wkbCircularString with Z component. ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCompoundCurveZ = 1009,   /**< wkbCompoundCurve with Z component. ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCurvePolygonZ = 1010,    /**< wkbCurvePolygon with Z component. ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbMultiCurveZ = 1011,      /**< wkbMultiCurve with Z component. ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbMultiSurfaceZ = 1012,    /**< wkbMultiSurface with Z component. ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbPoint25D = 0x80000001, /**< 2.5D extension as per 99-402 */
    wkbLineString25D = 0x80000002, /**< 2.5D extension as per 99-402 */
    wkbPolygon25D = 0x80000003, /**< 2.5D extension as per 99-402 */
    wkbMultiPoint25D = 0x80000004, /**< 2.5D extension as per 99-402 */
    wkbMultiLineString25D = 0x80000005, /**< 2.5D extension as per 99-402 */
    wkbMultiPolygon25D = 0x80000006, /**< 2.5D extension as per 99-402 */
    wkbGeometryCollection25D = 0x80000007 /**< 2.5D extension as per 99-402 */
}

enum int wkbCurve = 13; //TODO: Need to verify this.
enum int wkbSurface = 14; //TODO: Need to verify this.

extern(C) enum OGRwkbVariant
{
    wkbVariantOldOgc, /**< Old-style 99-402 extended dimension (Z) WKB types */
    wkbVariantIso,    /**< SFSQL 1.2 and ISO SQL/MM Part 3 extended dimension (Z&M) WKB types */
    wkbVariantPostGIS1/**< PostGIS 1.X has different codes for CurvePolygon, MultiCurve and MultiSurface */ 
}

extern(C) enum OGRwkbByteOrder
{
    wkbXDR = 0,      /* MSB/Sun/Motoroloa: Most Significant Byte First   */
    wkbNDR = 1       /* LSB/Intel/Vax: Least Significant Byte First      */
}


extern(C) struct OGREnvelope //From ogr_core.h
{
    double      MinX;
    double      MaxX;
    double      MinY;
    double      MaxY;
}

extern(C) struct OGREnvelope3D
{
    double      MinX;
    double      MaxX;
    double      MinY;
    double      MaxY;
    double      MinZ;
    double      MaxZ;
}

extern(C) void* OGRMalloc(  size_t );
extern(C) void* OGRCalloc(  size_t, size_t );
extern(C) void* OGRRealloc( void*, size_t );
extern(C) char* OGRStrdup(  const(char)* );
extern(C) void  OGRFree(    void* );

alias OGRErr = int;

enum int OGRERR_NONE 			= 0;
enum int OGRERR_NOT_ENOUGH_DATA     	= 1;  /* not enough data to deserialize */
enum int OGRERR_NOT_ENOUGH_MEMORY   	= 2;
enum int OGRERR_UNSUPPORTED_GEOMETRY_TYPE = 3;
enum int OGRERR_UNSUPPORTED_OPERATION 	= 4;
enum int OGRERR_CORRUPT_DATA        	= 5;
enum int OGRERR_FAILURE             	= 6;
enum int OGRERR_UNSUPPORTED_SRS     	= 7;
enum int OGRERR_INVALID_HANDLE      	= 8;
enum int OGRERR_NON_EXISTING_FEATURE 	= 9;  /* added in GDAL 2.0 */

alias OGRBoolean = int;

extern(C) enum OGRFieldType
{
  OFTInteger = 0,         /** Simple 32bit integer */ 
  OFTIntegerList = 1,     /** List of 32bit integers */ 
  OFTReal = 2,            /** Double Precision floating point */ 
  OFTRealList = 3,        /** List of doubles */    
  OFTString = 4,          /** String of ASCII chars */ 
  OFTStringList = 5,      /** Array of strings */ 
  OFTWideString = 6,      /** deprecated */ 
  OFTWideStringList = 7,  /** deprecated */ 
  OFTBinary = 8,          /** Raw Binary data */ 
  OFTDate = 9,            /** Date */ 
  OFTTime = 10,           /** Time */ 
  OFTDateTime = 11,       /** Date and Time */ 
  OFTInteger64 = 12,      /** Single 64bit integer */ 
  OFTInteger64List = 13,  /** List of 64bit integers */    
  OFTMaxType = 13
}

extern(C) enum OGRFieldSubType
{
   OFSTNone = 0,    /** No subtype. This is the default value */        
   OFSTBoolean = 1, /** Boolean integer. Only valid for OFTInteger and OFTIntegerList.*/
   OFSTInt16 = 2,   /** Signed 16-bit integer. Only valid for OFTInteger and 
                        OFTIntegerList. */                                                  
   OFSTFloat32 = 3, /** Single precision (32 bit) floatint point. Only valid for
                        OFTReal and OFTRealList. */                                                    
   OFSTMaxSubType = 3                                                                                                              
}

extern(C) enum OGRJustification
{
    OJUndefined = 0,
    OJLeft = 1,
    OJRight = 2
}

extern(C) union OGRField
{
    int         Integer;
    GIntBig     Integer64;
    double      Real;
    char*       String;
    
    struct _IntegerList 
    {
        int     nCount;
        int*    paList;
    }
    _IntegerList IntegerList;
    
    struct _Integer64List
    {
        int      nCount;
        GIntBig* paList;
    }
    _Integer64List Integer64List;

    struct _RealList
    {
        int     nCount;
        double* paList;
    }
    _RealList RealList;
    
    struct _StringList
    {
        int     nCount;
        char**  paList;
    }
    _StringList StringList;

    struct _Binary
    {
        int     nCount;
        GByte*  paData;
    }
    _Binary Binary;
    
    struct _Set
    {
        int     nMarker1;
        int     nMarker2;
    }
    _Set Set;

    struct _Date
    {
        GInt16  Year;
        GByte   Month;
        GByte   Day;
        GByte   Hour;
        GByte   Minute;
        GByte   TZFlag; /* 0=unknown, 1=localtime(ambiguous), 
                           100=GMT, 104=GMT+1, 80=GMT-5, etc */
        GByte   Reserved; /* must be set to 0 */
        float   Second; /* with millisecond accuracy. at the end of the 
                        structure, so as to keep it 12 bytes on 32 bit */
    }
    _Date Date;
}


extern(C) enum OGRSTClassId
{
    OGRSTCNone   = 0,
    OGRSTCPen    = 1,
    OGRSTCBrush  = 2,
    OGRSTCSymbol = 3,
    OGRSTCLabel  = 4,
    OGRSTCVector = 5
}

extern(C) enum OGRSTUnitId
{
    OGRSTUGround = 0,
    OGRSTUPixel  = 1,
    OGRSTUPoints = 2,
    OGRSTUMM     = 3,
    OGRSTUCM     = 4,
    OGRSTUInches = 5
}
