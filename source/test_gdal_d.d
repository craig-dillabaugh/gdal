import gdal;

/*
 * Test functions for GDAL Geospatial Data Abstraction Library. 
 *
 * Currently compiles with: dmd test_gdal_d.d gdal.d -L-ldgal
 * 
 */

/*
 * This function tests creating files. It creates three TIFF datasets with
 * different data types.
 */
bool testGDALCreate()
{
  import std.file, std.string;
  
  GDALAllRegister();

  GDALDriverH driver_handle = GDALIdentifyDriver("july_22_2007.tif", null);
  string file = "./testfile";
  int x_size = 32;
  int y_size = 32;
  int n_bands = 2;
  
  GDALCreate( driver_handle, toStringz( file ~ "byte.tif" ), x_size, y_size, n_bands, 
	      GDALDataType.GDT_Byte, null);
  GDALCreate( driver_handle, toStringz( file ~ "float.tif" ), x_size, y_size, n_bands, 
	      GDALDataType.GDT_Float32, null);
  GDALCreate( driver_handle, toStringz( file ~ "cint32.tif" ), x_size, y_size, n_bands, 
	      GDALDataType.GDT_CInt32, null);

  remove( file ~ "byte.tif");
  remove( file ~ "float.tif");
  remove( file ~ "cint32.tif");
  return true;
}

void testOpen()
{
  import std.stdio;
  import std.string;
  
  string testfile = "oct_13_2004_ikonos.tif";
  
  GDALDatasetH ds = GDALOpen( toStringz("oct_13_2004_ikonos.tif"), GDALAccess.GA_ReadOnly );
  
  int num_bands = GDALGetRasterCount( ds );
  int x_size = GDALGetRasterXSize( ds );
  int y_size = GDALGetRasterYSize( ds );
  
  writeln(testfile, " has ", num_bands, " bands and is [", x_size, "x", y_size, "]");
  
  /* Band index's start at 1. */
  GDALRasterBandH band = GDALGetRasterBand( ds, 1 );
  
  /* Getting Block Size */
  int x_dim, y_dim;
  GDALGetBlockSize( band, &x_dim, &y_dim );
  
  write("Band 1 is of size ", GDALGetRasterBandXSize( band ), " by ");
  writeln( GDALGetRasterBandYSize( band ), ".");
  writeln("\twith block size = ", x_dim, ", ", y_dim, ".");
  
  GDALClose( ds );
}


void main( string[] args )
{
  import std.stdio;
  
  testGDALCreate();
  testOpen();
  
  writeln("Welcome to gdal.d");
}