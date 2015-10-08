/*
 * D Bindings for the Geospatial Data Abstraction Library (GDAL) C Library
 * Version 2.0
 *
 * Binding Author: Craig Dillabaugh
 *
 * For more information on GDAL itself see:  http://www.gdal.org/
 *
 *
 */
module cpl_string;

import gdal;
 
import std.c.stdio;
import std.c.stdarg;
 
extern(C) char**       CSLAddString(char** papszStrList, const(char)* pszNewString);
extern(C) int          CSLCount(char** papszStrList);
extern(C) const(char)* CSLGetField( char**, int );
extern(C) void         CSLDestroy(char** papszStrList);
extern(C) char**       CSLDuplicate(char** papszStrList);
extern(C) char**       CSLMerge( char** papszOrig, char** papszOverride );

extern(C) char**       CSLTokenizeString(const(char)* pszString );
extern(C) char**       CSLTokenizeStringComplex( const(char)* pszString,
                                       const(char)* pszDelimiter,
                                       int bHonourStrings, 
                                       int bAllowEmptyTokens );
extern(C) char**       CSLTokenizeString2( const(char)* pszString, 
                                 const(char)* pszDelimeter, 
                                 int nCSLTFlags );
                                 
enum CSLT_HONOURSTRINGS      = 0x0001;
enum CSLT_ALLOWEMPTYTOKENS   = 0x0002;
enum CSLT_PRESERVEQUOTES     = 0x0004;
enum CSLT_PRESERVEESCAPES    = 0x0008;
enum CSLT_STRIPLEADSPACES    = 0x0010;
enum CSLT_STRIPENDSPACES     = 0x0020;

extern(C) int    CSLPrint(char** papszStrList, FILE *fpOut);
extern(C) char** CSLLoad(const(char)* pszFname) ;
extern(C) char** CSLLoad2(const(char)* pszFname, int nMaxLines, int nMaxCols, char** papszOptions) ;
extern(C) int    CSLSave(char **papszStrList, const(char)* pszFname);

extern(C) char** CSLInsertStrings(char **papszStrList, int nInsertAtLineNo, 
                         char **papszNewLines) ;
extern(C) char** CSLInsertString(char **papszStrList, int nInsertAtLineNo, 
                               const(char)* pszNewLine) ;
extern(C) char** CSLRemoveStrings(char **papszStrList, int nFirstLineToDelete,
                         int nNumToRemove, char ***ppapszRetStrings) ;
extern(C) int    CSLFindString( char **, const(char)*  );
extern(C) int    CSLFindStringCaseSensitive( char **, const(char)*  );
extern(C) int    CSLPartialFindString( char **papszHaystack, 
	                               const(char)*  pszNeedle );
extern(C) int    CSLFindName(char **papszStrList, const(char)* pszName);
extern(C) int    CSLTestBoolean( const(char)* pszValue );
extern(C) int    CSLFetchBoolean( char **papszStrList, const(char)* pszKey, 
                             int bDefault );

extern(C) const(char)* CPLParseNameValue(const(char)* pszNameValue, char **ppszKey );
extern(C) const(char)* CSLFetchNameValue(char **papszStrList, const(char)* pszName);
extern(C) const(char)* CSLFetchNameValueDef(char **papszStrList, const(char)* pszName,
                           const(char)* pszDefault );
extern(C) char**       CSLFetchNameValueMultiple(char **papszStrList, const(char)* pszName);
extern(C) char**       CSLAddNameValue(char **papszStrList, 
                                       const(char)* pszName, const(char)* pszValue) ;
extern(C) char**       CSLSetNameValue(char** papszStrList, 
                                       const(char)* pszName, const(char)* pszValue) ;
extern(C) void  CSLSetNameValueSeparator( char ** papszStrList, 
                                       const(char)* pszSeparator );

enum CPLES_BackslashQuotable = 0;
enum CPLES_XML               = 1;
enum CPLES_URL               = 2;
enum CPLES_SQL               = 3;
enum CPLES_CSV               = 4;
enum CPLES_XML_BUT_QUOTES    = 5;

extern(C) char  *CPLEscapeString( const(char)* pszString, int nLength, 
                                  int nScheme ) ;
extern(C) char  *CPLUnescapeString( const(char)* pszString, int *pnLength,
                                    int nScheme ) ;

extern(C) char  *CPLBinaryToHex( int nBytes, const(GByte)* pabyData ) ;
extern(C) GByte  *CPLHexToBinary( const(char)* pszHex, int *pnBytes ) ;

extern(C) char  *CPLBase64Encode( int nBytes, const(GByte)* pabyData ) ;
extern(C) int  CPLBase64DecodeInPlace(GByte* pszBase64);

enum CPLValueType
{
    CPL_VALUE_STRING,
    CPL_VALUE_REAL,
    CPL_VALUE_INTEGER
}

extern(C) CPLValueType CPLGetValueType(const char* pszValue);

extern(C) size_t  CPLStrlcpy(char* pszDest, const char* pszSrc, size_t nDestSize);
extern(C) size_t  CPLStrlcat(char* pszDest, const char* pszSrc, size_t nDestSize);
extern(C) size_t  CPLStrnlen (const(char)* pszStr, size_t nMaxLen);

/* -------------------------------------------------------------------- */
/*      Locale independant formatting functions.                        */
/* -------------------------------------------------------------------- */
extern(C) int  CPLvsnprintf(char *str, size_t size, const char* fmt, va_list args);
extern(C) int  CPLsnprintf(char *str, size_t size, const char* fmt, ...); // CPL_PRINT_FUNC_FORMAT(3,4);
extern(C) int  CPLsprintf(char *str, const char* fmt, ...); // CPL_PRINT_FUNC_FORMAT(2, 3);
extern(C) int  CPLprintf(const char* fmt, ...); // CPL_PRINT_FUNC_FORMAT(1, 2);
extern(C) int  CPLsscanf(const char* str, const char* fmt, ...); /* caution: only works with limited number of formats */

extern(C) const(char)* CPLSPrintf(const(char)* fmt, ...); // CPL_PRINT_FUNC_FORMAT(1, 2);
extern(C) char**       CSLAppendPrintf(char **papszStrList, const(char)* fmt, ...); // CPL_PRINT_FUNC_FORMAT(2, 3) CPL_WARN_UNUSED_RESULT;
extern(C) int          CPLVASPrintf(char **buf, const(char)* fmt, va_list args );

/* -------------------------------------------------------------------- */
/*      RFC 23 character set conversion/recoding API (cpl_recode.cpp).  */
/* -------------------------------------------------------------------- */
enum const(char)* CPL_ENC_LOCALE    = "";
enum const(char)* CPL_ENC_UTF8      = "UTF-8";
enum const(char)* CPL_ENC_UTF16     = "UTF-16";
enum const(char)* CPL_ENC_UCS2      = "UCS-2";
enum const(char)* CPL_ENC_UCS4      = "UCS-4";
enum const(char)* CPL_ENC_ASCII     = "ASCII";
enum const(char)* CPL_ENC_ISO8859_1 = "ISO-8859-1";

extern(C) int       CPLEncodingCharSize( const(char)* pszEncoding );
extern(C) void      CPLClearRecodeWarningFlags( );
extern(C) char*     CPLRecode( const(char)* pszSource, 
                         const(char)* pszSrcEncoding, 
                         const(char)* pszDstEncoding ); // CPL_WARN_UNUSED_RESULT;
extern(C) char*     CPLRecodeFromWChar( const(wchar)* pwszSource, 
                                  const(char)* pszSrcEncoding, 
                                  const(char)* pszDstEncoding ); // CPL_WARN_UNUSED_RESULT;
extern(C) wchar*  CPLRecodeToWChar( const(char)* pszSource,
                                   const(char)* pszSrcEncoding, 
                                   const(char)* pszDstEncoding ); // CPL_WARN_UNUSED_RESULT;
extern(C) int       CPLIsUTF8(const char* pabyData, int nLen);
extern(C) char*     CPLForceToASCII(const char* pabyData, int nLen, char chReplacementChar); // CPL_WARN_UNUSED_RESULT;
extern(C) int       CPLStrlenUTF8(const(char)* pszUTF8Str);