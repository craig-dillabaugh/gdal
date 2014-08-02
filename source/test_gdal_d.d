import gdal;

/*
 * This function tests creating files. It creates three TIFF datasets with
 * different data types.
 */
bool testGDALCreate()
{
  import std.string;
  
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
  GDALClose( ds );
}


void main( string[] args )
{
  import std.stdio;
  
  testGDALCreate();
  testOpen();
  
  writeln("Welcome to gdal.d");
}