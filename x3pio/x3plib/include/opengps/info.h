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
 * Copyright and license information.
 */

#ifndef _OPENGPS_INFO_H
#define _OPENGPS_INFO_H

#ifndef _OPENGPS_H
#  include <opengps/opengps.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

   /*!
    * Gets a short name identifying this software library.
    *
    * @param text Allocated string buffer. Will contain the resulting message on success.
    * @param size The size of the allocated buffer in characters.
    * @returns On success returns the number of characters written, otherwise returns the
    * required size in characters of an allocated text buffer to store the name including
    * the terminating null character.
    */
   _OPENGPS_EXPORT size_t ogps_GetNameInfo(OGPS_Character* const text, const size_t size);

   /*!
    * Gets a short message describing the purpose of this software library.
    *
    * @param text Allocated string buffer. Will contain the resulting message on success.
    * @param size The size of the allocated buffer in characters.
    * @returns On success returns the number of characters written, otherwise returns the
    * required size  in characters of an allocated text buffer to store the about message
    * including the terminating null character.
    */
   _OPENGPS_EXPORT size_t ogps_GetAboutInfo(OGPS_Character* const text, const size_t size);

   /*!
    * Gets the version identifier of this software library.
    *
    * @param text Allocated string buffer. Will contain the resulting message on success.
    * @param size The size of the allocated buffer in characters.
    * @returns On success returns the number of characters written, otherwise returns the
    * required size  in characters of an allocated text buffer to store the version
    * identifier including the terminating null character.
    */
   _OPENGPS_EXPORT size_t ogps_GetVersionInfo(OGPS_Character* const text, const size_t size);

   /*!
    * Gets the copyright information of this software library.
    *
    * @param text Allocated string buffer. Will contain the resulting message on success.
    * @param size The size of the allocated buffer in characters.
    * @returns On success returns the number of characters written, otherwise returns the
    * required size  in characters of an allocated text buffer to store the copyright
    * information including the terminating null character.
    */
   _OPENGPS_EXPORT size_t ogps_GetCopyrightInfo(OGPS_Character* const text, const size_t size);

   /*!
    * Gets the license information of this software library.
    *
    * @param text Allocated string buffer. Will contain the resulting message on success.
    * @param size The size of the allocated buffer in characters.
    * @returns On success returns the number of characters written, otherwise returns the
    * required size  in characters of an allocated text buffer to store the license
    * information including the terminating null character.
    */
   _OPENGPS_EXPORT size_t ogps_GetLicenseInfo(OGPS_Character* const text, const size_t size);

   /*!
    * Prints the version identifier of this software library to standard output.
    */
   _OPENGPS_EXPORT void ogps_PrintVersionInfo();


   /*!
    * Prints the copyright information of this software library to standard output.
    */
   _OPENGPS_EXPORT void ogps_PrintCopyrightInfo();

   /*!
    * Prints the license information of this software library to standard output.
    */
   _OPENGPS_EXPORT void ogps_PrintLicenseInfo();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif    /* _OPENGPS_INFO_H */
/*! @} */
