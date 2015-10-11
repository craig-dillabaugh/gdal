/*
 * D Bindings for the Geospatial Data Abstraction Library (GDAL) C Library
 * Version 2.0
 *
 * Binding Author: Craig Dillabaugh
 *
 * For more information on GDAL itself see:  http://www.gdal.org/
 *
 * Based on:
 *  ogr_api.h 28900 2015-04-14 09:40:34Z rouault $
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  C API for OGR Geometry, Feature, Layers, DataSource and drivers.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************



#include "cpl_progress.h"
#include "cpl_minixml.h"
#include "ogr_core.h"

CPL_C_START

/* -------------------------------------------------------------------- */
/*      Geometry related functions (ogr_geometry.h)                     */
/* -------------------------------------------------------------------- */
module ogr_api;

import gdal;

import std.c.stdio;


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
