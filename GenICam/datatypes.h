/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#ifndef DATATYPES_H
#define DATATYPES_H

#define NOMINMAX

#include "GenApi/GenApi.h"
#include "common/sharedStructuresQt.h"
#include "common/paramMeta.h"

#include "GenTL_v1_5.h"

#include <qmap.h>
#include <qstring.h>

using namespace GENAPI_NAMESPACE;

//------------------------------------------------------------------------
class GCType
{
public:
    GCType(QMap<QString, ito::Param> *paramMap, const QString &name);
    virtual ~GCType();

    ito::Param &param() const { return (*m_paramMap)[m_name]; }
    virtual ito::RetVal setValue(const ito::ParamBase *value) = 0;
    virtual INode* node() const = 0;
    virtual ito::RetVal update(bool valueOnly = true) = 0; //if valueOnly = false, the meta information is updated, too

protected:
    int flagsFromAccessMode(const GenApi::EAccessMode &accessMode) const;
    QMap<QString, ito::Param> *m_paramMap;
    QString m_name;
};

//------------------------------------------------------------------------
class GCIntType : public GCType
{
public:
    GCIntType(QMap<QString, ito::Param> *paramMap, const QString &name, const CIntegerPtr &ptr);
    ~GCIntType();
    CIntegerPtr value() const { return m_sharedPtr;  }
    INode *node() const;
    ito::RetVal setValue(const ito::ParamBase *value);
    ito::RetVal update(bool valueOnly = true);

private:
    void intMetaFromInteger(const CIntegerPtr &iPtr, ito::IntMeta *intMeta) const;
    CIntegerPtr m_sharedPtr;
};

//------------------------------------------------------------------------
class GCFloatType : public GCType
{
public:
    GCFloatType(QMap<QString, ito::Param> *paramMap, const QString &name, const CFloatPtr &ptr);
    ~GCFloatType();
    CFloatPtr value() const { return m_sharedPtr; }
    INode *node() const;
    ito::RetVal setValue(const ito::ParamBase *value);
    ito::RetVal update(bool valueOnly = true);

private:
    void doubleMetaFromFloat(const CFloatPtr &fPtr, ito::DoubleMeta *dblMeta) const;
    CFloatPtr m_sharedPtr;
};

//------------------------------------------------------------------------
class GCStringType : public GCType
{
public:
    GCStringType(QMap<QString, ito::Param> *paramMap, const QString &name, const CStringPtr &ptr);
    ~GCStringType();
    INode *node() const;
    CStringPtr value() const { return m_sharedPtr; }
    ito::RetVal setValue(const ito::ParamBase *value);
    ito::RetVal update(bool valueOnly = true);

private:
    CStringPtr m_sharedPtr;
};

//------------------------------------------------------------------------
class GCBoolType : public GCType
{
public:
    GCBoolType(QMap<QString, ito::Param> *paramMap, const QString &name, const CBooleanPtr &ptr);
    ~GCBoolType();
    INode *node() const;
    CBooleanPtr value() const { return m_sharedPtr; }
    ito::RetVal setValue(const ito::ParamBase *value);
    ito::RetVal update(bool valueOnly = true);

private:
    CBooleanPtr m_sharedPtr;
};

//------------------------------------------------------------------------
class GCEnumerationType : public GCType
{
public:
    GCEnumerationType(QMap<QString, ito::Param> *paramMap, const QString &name, const CEnumerationPtr &ptr);
    ~GCEnumerationType();
    INode *node() const;
    CEnumerationPtr value() const { return m_sharedPtr; }
    ito::RetVal setValue(const ito::ParamBase *value);
    ito::RetVal update(bool valueOnly = true);

private:
    void stringMetaFromEnumeration(const CEnumerationPtr &ePtr, ito::StringMeta *strMeta) const;

    CEnumerationPtr m_sharedPtr;
};




#endif // DATATYPES_H
