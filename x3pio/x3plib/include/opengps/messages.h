/***************************************************************************
 *   Copyright by Johannes Herwig (NanoFocus AG) 2007                      *
 *   Copyright by Georg Wiora (NanoFocus AG) 2007                          *
 *                                                                         *
 *   This file is part of the openGPS (R)[TM] software library.            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License (LGPL)    *
 *   as published by the Free Software Foundation; either version 3 of     *
 *   the License, or (at your option) any later version.                   *
 *   for detail see the files "licence_LGPL-3.0.txt" and                   *
 *   "licence_GPL-3.0.txt".                                                *
 *                                                                         *
 *   openGPS is distributed in the hope that it will be useful,            *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *                                                                         *
 *   The name "openGPS" and the logo are registered as                     *
 *   European trade mark No. 006178354 for                                 *
 *   Physikalisch Technische Bundesanstalt (PTB)                           *
 *   http://www.ptb.de/                                                    *
 *                                                                         *
 *   More information about openGPS can be found at                        *
 *   http://www.opengps.eu/                                                *
 ***************************************************************************/

/*! \addtogroup C
 *  @{
 */

/*! @file
 * Global handling of error and warning messages. This wrappes C++ exception
 * handling to be usable in the C interface. See OpenGPS::Exception for
 * details about throwing exceptions in this software library.
 */

#ifndef _OPENGPS_MESSAGES_H
#define _OPENGPS_MESSAGES_H

#ifndef _OPENGPS_H
#  include <opengps/opengps.h>
#endif

/*!
 * Possible failure conditions.
 */
typedef enum _OPENGPS_EXCEPTION_ID
{
   /*! No failure condition trapped. This serves as some kind of default value. */
   OGPS_ExNone,
   /*! A failure condition occured, but it has not been specified in detail. */
   OGPS_ExGeneral,
   /*! The value of at least one of the parameters passed to a function is invalid in the current context. */
   OGPS_ExInvalidArgument,
   /*! Due to the state of the object an operation could not be performed. */
   OGPS_ExInvalidOperation,
   /*! A specific implementation of an interface does not implement this operation. */
   OGPS_ExNotImplemented,
   /*! An overflow occured. There is no guarantee of the integrity of further processing steps. */
   OGPS_ExOverflow,
   /*! Indicates a non-fatal error that may be ignored. This is for informational purpose only. */
   OGPS_ExWarning
} OGPS_ExceptionId; /*! Possible failure conditions. */

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

   /*!
    * Gets the last handled error message or NULL.
    *
    * @remarks This may return NULL even though an error occured. This is because
    * the description of a particular error condition may not be available.
    *
    * Use ::ogps_HasError to check whether an error condition has been trapped.
    *
    * @see ::ogps_GetErrorId
    */
   _OPENGPS_EXPORT const OGPS_Character* ogps_GetErrorMessage();

   /*!
    * Gets the last handled detailed error description or NULL.
    *
    * @remarks This may return NULL even though an error occured. This is because
    * the description of a particular error condition may not be available.
    *
    * @see ::ogps_GetErrorId
    */
   _OPENGPS_EXPORT const OGPS_Character* ogps_GetErrorDescription();

   /*!
    * Gets the type of the last handled error condition.
    *
    * @see ::ogps_GetErrorMessage
    */
   _OPENGPS_EXPORT OGPS_ExceptionId ogps_GetErrorId();

   /*!
    * Returns TRUE when the statement executed previously failed.
    * Use ::ogps_GetErrorId to obtain more information about the failure.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_HasError();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif    /* _OPENGPS_MESSAGES_H */
/*! @} */
