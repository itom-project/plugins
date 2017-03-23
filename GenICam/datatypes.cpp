/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#include "datatypes.h"

#include "gccommon.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include <iostream>

#include <Base/GCBase.h>
#include <GenApi/GenApi.h>
#include "PFNC.h"

//###########################################################################
//###########################################################################
//---------------------------------------------------------------------------
GCType::GCType(QMap<QString, ito::Param> *paramMap, const QString &name) :
m_paramMap(paramMap),
m_name(name)
{
	
}

//---------------------------------------------------------------------------
GCType::~GCType()
{

}

//--------------------------------------------------------------------------------------------------------
int GCType::flagsFromAccessMode(const GenApi::EAccessMode &accessMode) const
{
	int flag = 0;
	switch (accessMode)
	{
	case EAccessMode::NA:
		flag = ito::ParamBase::Readonly | ito::ParamBase::NotAvailable;
		break;
	case EAccessMode::RO:
		flag = ito::ParamBase::Readonly;
		break;
	case EAccessMode::RW:
		flag = 0;
		break;
	case EAccessMode::WO:
		flag = 0; //todo: good?
		break;
	default:
		flag = 0;
	}

	return flag;
}

//###########################################################################
//###########################################################################
//---------------------------------------------------------------------------
GCIntType::GCIntType(QMap<QString, ito::Param> *paramMap, const QString &name, const CIntegerPtr &ptr) :
	GCType(paramMap, name),
	m_sharedPtr(ptr)
{

}

//---------------------------------------------------------------------------
GCIntType::~GCIntType()
{

}

//---------------------------------------------------------------------------
INode* GCIntType::node() const
{
	return m_sharedPtr->GetNode();
}

//---------------------------------------------------------------------------
ito::RetVal GCIntType::setValue(const ito::ParamBase *value)
{
	if (!value || value->getType() != ito::ParamBase::Int)
	{
		return ito::RetVal::format(ito::retError, 0, "Invalid parameter. Integer data type requested for device parameter %s", m_sharedPtr->GetNode()->GetName().c_str());
	}
	else
	{
		m_sharedPtr->SetValue(value->getVal<int>());
		return update(true);
	}
}

//------------------------------------------------------------------------------------------------
void GCIntType::intMetaFromInteger(const CIntegerPtr &iPtr, ito::IntMeta *intMeta) const
{

	int64_t minimum = qBound((int64_t)INT_MIN, iPtr->GetMin(), (int64_t)INT_MAX);
	int64_t maximum = qBound((int64_t)INT_MIN, iPtr->GetMax(), (int64_t)INT_MAX);

	if (minimum == -1 && maximum == -1)
	{
		intMeta->setMin(std::numeric_limits<int>::min());
		intMeta->setMax(std::numeric_limits<int>::max());
		intMeta->setStepSize(1);
	}
	else if (iPtr->GetIncMode() == EIncMode::noIncrement)
	{
		intMeta->setMin(minimum);
		intMeta->setMax(maximum);
		intMeta->setStepSize(1);
	}
	else if (iPtr->GetIncMode() == EIncMode::fixedIncrement)
	{
		intMeta->setMin(minimum);
		intMeta->setMax(maximum);
		intMeta->setStepSize(iPtr->GetInc());
	}
	else
	{
		intMeta->setMin(minimum);
		intMeta->setMax(maximum);
	}

	switch (iPtr->GetRepresentation())
	{
	case GenApi::Linear:
		intMeta->setRepresentation(ito::ParamMeta::Linear);
		break;
	case GenApi::Logarithmic:
		intMeta->setRepresentation(ito::ParamMeta::Logarithmic);
		break;
	case GenApi::Boolean:
		intMeta->setRepresentation(ito::ParamMeta::Boolean);
		break;
	case GenApi::PureNumber:
		intMeta->setRepresentation(ito::ParamMeta::PureNumber);
		break;
	case GenApi::HexNumber:
		intMeta->setRepresentation(ito::ParamMeta::HexNumber);
		break;
	case GenApi::IPV4Address:
		intMeta->setRepresentation(ito::ParamMeta::IPV4Address);
		break;
	case GenApi::MACAddress:
		intMeta->setRepresentation(ito::ParamMeta::MACAddress);
		break;
	default:
		intMeta->setRepresentation(ito::ParamMeta::UnknownRepresentation);
		break;
	}

	intMeta->setUnit(iPtr->GetUnit().c_str());
}

//------------------------------------------------------------------------------------------------
ito::RetVal GCIntType::update(bool valueOnly /*= true*/)
{
	ito::RetVal retval;
	int flags = flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode());
	if ((!(flags & ito::ParamBase::NotAvailable)))
	{
		retval = (*m_paramMap)[m_name].setVal<int>(m_sharedPtr->GetValue());
	}

	if (!valueOnly)
	{
        intMetaFromInteger(m_sharedPtr, (*m_paramMap)[m_name].getMetaT<ito::IntMeta>());
        (*m_paramMap)[m_name].setFlags(flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode()));
	}

	return retval;
}

//###########################################################################
//###########################################################################
//---------------------------------------------------------------------------
GCFloatType::GCFloatType(QMap<QString, ito::Param> *paramMap, const QString &name, const CFloatPtr &ptr) :
    GCType(paramMap, name),
    m_sharedPtr(ptr)
{

}

//---------------------------------------------------------------------------
GCFloatType::~GCFloatType()
{

}

//---------------------------------------------------------------------------
INode* GCFloatType::node() const
{
	return m_sharedPtr->GetNode();
}

//---------------------------------------------------------------------------
ito::RetVal GCFloatType::setValue(const ito::ParamBase *value)
{
	if (!value || value->getType() != ito::ParamBase::Double)
	{
		return ito::RetVal::format(ito::retError, 0, "Invalid parameter. Float data type requested for device parameter %s", m_sharedPtr->GetNode()->GetName().c_str());
	}
	else
	{
		m_sharedPtr->SetValue(value->getVal<ito::float64>());
		return update(true);
	}
}

//------------------------------------------------------------------------------------------------
void GCFloatType::doubleMetaFromFloat(const CFloatPtr &fPtr, ito::DoubleMeta *dblMeta) const
{
	ito::float64 minimum = fPtr->GetMin();
	ito::float64 maximum = fPtr->GetMax();

	if (minimum == -1 && maximum == -1)
	{
		dblMeta->setMin(-std::numeric_limits<ito::float64>::max());
		dblMeta->setMax(std::numeric_limits<ito::float64>::max());
		dblMeta->setStepSize(0.0);
	}
	else if (fPtr->GetIncMode() == EIncMode::noIncrement)
	{
		dblMeta->setMin(minimum);
		dblMeta->setMax(maximum);
		dblMeta->setStepSize(0.0);
	}
	else if (fPtr->GetIncMode() == EIncMode::fixedIncrement)
	{
		dblMeta->setMin(minimum);
		dblMeta->setMax(maximum);
		dblMeta->setStepSize(fPtr->GetInc());
	}
	else
	{
		dblMeta->setMin(minimum);
		dblMeta->setMax(maximum);
	}

	switch (fPtr->GetDisplayNotation())
	{
		case GenApi::fnAutomatic:
			dblMeta->setDisplayNotation(ito::DoubleMeta::Automatic);
			break;
		case GenApi::fnFixed:
			dblMeta->setDisplayNotation(ito::DoubleMeta::Fixed);
			break;
		case GenApi::fnScientific:
			dblMeta->setDisplayNotation(ito::DoubleMeta::Scientific);
			break;	
	}

	switch (fPtr->GetRepresentation())
	{
	case GenApi::Linear:
		dblMeta->setRepresentation(ito::ParamMeta::Linear);
		break;
	case GenApi::Logarithmic:
		dblMeta->setRepresentation(ito::ParamMeta::Logarithmic);
		break;
	case GenApi::Boolean:
		dblMeta->setRepresentation(ito::ParamMeta::Boolean);
		break;
	case GenApi::PureNumber:
		dblMeta->setRepresentation(ito::ParamMeta::PureNumber);
		break;
	case GenApi::HexNumber:
		dblMeta->setRepresentation(ito::ParamMeta::HexNumber);
		break;
	case GenApi::IPV4Address:
		dblMeta->setRepresentation(ito::ParamMeta::IPV4Address);
		break;
	case GenApi::MACAddress:
		dblMeta->setRepresentation(ito::ParamMeta::MACAddress);
		break;
	default:
		dblMeta->setRepresentation(ito::ParamMeta::UnknownRepresentation);
		break;
	}

	dblMeta->setDisplayPrecision(fPtr->GetDisplayPrecision());
	dblMeta->setUnit(fPtr->GetUnit().c_str());
}

//---------------------------------------------------------------------------
ito::RetVal GCFloatType::update(bool valueOnly /*= true*/)
{
	ito::RetVal retval;
	int flags = flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode());
	if ((!(flags & ito::ParamBase::NotAvailable)))
	{
		retval = (*m_paramMap)[m_name].setVal<ito::float64>(m_sharedPtr->GetValue());
	}

	if (!valueOnly)
	{
		doubleMetaFromFloat(m_sharedPtr, (*m_paramMap)[m_name].getMetaT<ito::DoubleMeta>());
		(*m_paramMap)[m_name].setFlags(flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode()));
	}

	return retval;
}

//###########################################################################
//###########################################################################
//---------------------------------------------------------------------------
GCStringType::GCStringType(QMap<QString, ito::Param> *paramMap, const QString &name, const CStringPtr &ptr) :
    GCType(paramMap, name),
    m_sharedPtr(ptr)
{

}

//---------------------------------------------------------------------------
GCStringType::~GCStringType()
{

}

//---------------------------------------------------------------------------
INode* GCStringType::node() const
{
	return m_sharedPtr->GetNode();
}

//---------------------------------------------------------------------------
ito::RetVal GCStringType::setValue(const ito::ParamBase *value)
{
	if (!value || value->getType() != ito::ParamBase::String)
	{
		return ito::RetVal::format(ito::retError, 0, "Invalid parameter. String data type requested for device parameter %s", m_sharedPtr->GetNode()->GetName().c_str());
	}
	else
	{
		m_sharedPtr->SetValue(value->getVal<const char*>());
		return update(true);
	}
}

//---------------------------------------------------------------------------
ito::RetVal GCStringType::update(bool valueOnly /*= true*/)
{
	ito::RetVal retval;
	int flags = flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode());
	if ((!(flags & ito::ParamBase::NotAvailable)))
	{
		retval = (*m_paramMap)[m_name].setVal<const char*>(m_sharedPtr->GetValue().c_str());
	}

	if (!valueOnly)
	{
		(*m_paramMap)[m_name].setFlags(flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode()));
	}

	return retval;
}

//###########################################################################
//###########################################################################
//---------------------------------------------------------------------------
GCBoolType::GCBoolType(QMap<QString, ito::Param> *paramMap, const QString &name, const CBooleanPtr &ptr) :
    GCType(paramMap, name),
    m_sharedPtr(ptr)
{

}

//---------------------------------------------------------------------------
GCBoolType::~GCBoolType()
{

}

//---------------------------------------------------------------------------
INode* GCBoolType::node() const
{
	return m_sharedPtr->GetNode();
}

//---------------------------------------------------------------------------
ito::RetVal GCBoolType::setValue(const ito::ParamBase *value)
{
	if (!value || value->getType() != ito::ParamBase::Int)
	{
		return ito::RetVal::format(ito::retError, 0, "Invalid parameter. Int data type requested for device parameter %s", m_sharedPtr->GetNode()->GetName().c_str());
	}
	else
	{
		m_sharedPtr->SetValue(value->getVal<int>() ? true : false);
		return update(true);
	}
}

//---------------------------------------------------------------------------
ito::RetVal GCBoolType::update(bool valueOnly /*= true*/)
{
	ito::RetVal retval;
	int flags = flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode());
	if ((!(flags & ito::ParamBase::NotAvailable)))
	{
		retval = (*m_paramMap)[m_name].setVal<int>(m_sharedPtr->GetValue() ? 1 : 0);
	}

	if (!valueOnly)
	{
		(*m_paramMap)[m_name].setFlags(flags);
	}

	return retval;
}

//###########################################################################
//###########################################################################
//---------------------------------------------------------------------------
GCEnumerationType::GCEnumerationType(QMap<QString, ito::Param> *paramMap, const QString &name, const CEnumerationPtr &ptr) :
    GCType(paramMap, name),
    m_sharedPtr(ptr)
{

}

//---------------------------------------------------------------------------
GCEnumerationType::~GCEnumerationType()
{

}

//---------------------------------------------------------------------------
INode* GCEnumerationType::node() const
{
	return m_sharedPtr->GetNode();
}

//---------------------------------------------------------------------------
ito::RetVal GCEnumerationType::setValue(const ito::ParamBase *value)
{
	if (!value || value->getType() != ito::ParamBase::String)
	{
		return ito::RetVal::format(ito::retError, 0, "Invalid parameter. Int or string data type requested for device parameter %s", m_sharedPtr->GetNode()->GetName().c_str());
	}
	else
	{
		const char* val = value->getVal<const char*>();
		IEnumEntry *iee = m_sharedPtr->GetEntryByName(val);
		if (iee)
		{
			m_sharedPtr->SetIntValue(iee->GetValue());
		}
		else
		{
			return ito::RetVal::format(ito::retError, 0, "Invalid parameter. The enumeration entry '%s' does not exist.", val);
		}

		return update(true);
	}
}

//------------------------------------------------------------------------------------------------
void GCEnumerationType::stringMetaFromEnumeration(const CEnumerationPtr &ePtr, ito::StringMeta *strMeta) const
{
	strMeta->setStringType(ito::StringMeta::String);
	strMeta->clearItems();

	GenApi::NodeList_t entries;
	IEnumEntry *iee;
	ePtr->GetEntries(entries);

	for (size_t i = 0; i < entries.size(); ++i)
	{
		iee = dynamic_cast<IEnumEntry*>(entries[i]);
		if (iee)
		{
			strMeta->addItem(iee->GetSymbolic().c_str());
		}
	}
}

//-------------------------------------------------------------------------------------------------
ito::RetVal GCEnumerationType::update(bool valueOnly /*= true*/)
{
	ito::RetVal retval;
	int flags = flagsFromAccessMode(m_sharedPtr->GetNode()->GetAccessMode());
	if (!(flags & ito::ParamBase::NotAvailable))
	{
		IEnumEntry *iee = m_sharedPtr->GetCurrentEntry();
		if (iee)
		{
			retval += (*m_paramMap)[m_name].setVal<const char*>(iee->GetSymbolic().c_str());
		}
		else
		{
			retval += (*m_paramMap)[m_name].setVal<const char*>("");
		}
	}
	else
	{
		retval += (*m_paramMap)[m_name].setVal<const char*>("");
	}

	if (!valueOnly)
	{
		stringMetaFromEnumeration(m_sharedPtr, (*m_paramMap)[m_name].getMetaT<ito::StringMeta>());
		(*m_paramMap)[m_name].setFlags(flags);
	}
	
	return retval;
}
