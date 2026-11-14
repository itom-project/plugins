/* ********************************************************************
    Plugin "DummyMultiChannelGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#pragma once

#include <qobject.h>
#include <qrect.h>
#include "DataObject/dataobj.h"

class CameraEmulator : public QObject
{
public:
    explicit CameraEmulator();

    const ito::DataObject& imageMono() const { return m_imageMono; }
    const ito::DataObject& imageTopography() const { return m_imageTopography; }
    const ito::DataObject& imageColor() const { return m_imageColor; }

    void configureImageMono(const QRect &roi, int bpp);
    void configureImageTopography(const QRect& roi, bool singlePrecision);
    void configureImageColor(const QRect& roi, bool alpha);

    bool grabImages(bool imgMono, bool imgFloat, bool imgColor, float exposureTimeMs);

private:
    bool grabMono();
    bool grabTopography();
    bool grabColor();

    QRect m_roiMono;
    QRect m_roiTopography;
    QRect m_roiColor;
    ito::DataObject m_imageMono;
    ito::DataObject m_imageTopography;
    ito::DataObject m_imageColor;
    int m_imageMonoBpp;
    bool m_imageColorAlpha;
};
