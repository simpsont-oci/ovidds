
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl ovidds
// ------------------------------
#ifndef OVIDDS_EXPORT_H
#define OVIDDS_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (OVIDDS_HAS_DLL)
#  define OVIDDS_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && OVIDDS_HAS_DLL */

#if !defined (OVIDDS_HAS_DLL)
#  define OVIDDS_HAS_DLL 1
#endif /* ! OVIDDS_HAS_DLL */

#if defined (OVIDDS_HAS_DLL) && (OVIDDS_HAS_DLL == 1)
#  if defined (OVIDDS_BUILD_DLL)
#    define ovidds_Export ACE_Proper_Export_Flag
#    define OVIDDS_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define OVIDDS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* OVIDDS_BUILD_DLL */
#    define ovidds_Export ACE_Proper_Import_Flag
#    define OVIDDS_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define OVIDDS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* OVIDDS_BUILD_DLL */
#else /* OVIDDS_HAS_DLL == 1 */
#  define ovidds_Export
#  define OVIDDS_SINGLETON_DECLARATION(T)
#  define OVIDDS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* OVIDDS_HAS_DLL == 1 */

// Set OVIDDS_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (OVIDDS_NTRACE)
#  if (ACE_NTRACE == 1)
#    define OVIDDS_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define OVIDDS_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !OVIDDS_NTRACE */

#if (OVIDDS_NTRACE == 1)
#  define OVIDDS_TRACE(X)
#else /* (OVIDDS_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define OVIDDS_TRACE(X) ACE_TRACE_IMPL(X)
#  include "ace/Trace.h"
#endif /* (OVIDDS_NTRACE == 1) */

#endif /* OVIDDS_EXPORT_H */

// End of auto generated file.
