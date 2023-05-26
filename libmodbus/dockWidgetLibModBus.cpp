/* ********************************************************************
    Plugin "LibModBus" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "dockWidgetLibModBus.h"

#include "math.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------
char getHexChar(int i)
{
    if (i < 10)
    {
        return char(i + 48);
    }
    else
    {
        return char(i + 55);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetLibModBus::DockWidgetLibModBus(QMap<QString, ito::Param> params, int uniqueID)
{
    ui.setupUi(this);
    char* temp = params["name"].getVal<char*>(); //char* is borrowed reference, do not delete it
//    ui.lblName->setText(temp);
    ui.lblID->setText(QString::number(uniqueID));

    valuesChanged(params);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetLibModBus::valuesChanged(QMap<QString, ito::Param> params)
{
    if (params.keys().contains("debug"))
    {
        ui.groupBox_3->setEnabled(params["debug"].getVal<int>());
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/* void DockWidgetLibModBus::uniqueIDChanged(const int uniqueID)
{
    ui.lblID->setText(QString::number(uniqueID));
}*/

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLibModBus::on_ClrButton_clicked()
{
    ui.textTransfer->clear();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLibModBus::serialLog(QByteArray data, QByteArray endline, const char InOutChar)
{
    if (ui.groupBox_3->isEnabled())
    {
        QByteArray text1, text2, text3, prefix;
        int n = 0, i = 0, r1 = 0, r2 = 0, displayType = 0, displayLength = 0, uc = 0;

        prefix = "";
        displayLength = 1;
        if (ui.RBHex->isChecked())
        {
            displayLength = 2;
            displayType = 1;
            prefix = "0x ";
        }
        else if (ui.RBBin->isChecked())
        {
            displayLength = 8;
            displayType = 2;
        }
        else if (ui.RBDez->isChecked())
        {
            displayLength = 3;
            displayType = 3;
        }

        data = data + endline;

        text1.resize((displayLength+1) * data.length());
        text2.resize((3 * data.length()));

        for (n = 0; n < data.length(); n++)
        {
            uc = (int)(unsigned char)(data[n]);
            switch (displayType)
            {
                case 1: //If Hexagonal
                    {
                        text1[r1++] = getHexChar(int(uc / 16));
                        text1[r1++] = getHexChar(int(uc % 16));
                        text1[r1++] = ' ';
                        break;
                    }
                case 2: //If Binary
                    {
                        for (i = 7; i >= 0; i--)
                        {
                            text1[r1++] = ((uc >> i) & 1) + 48;
                        }
                        text1[r1++] = ' ';
                        break;
                    }
                case 3: //If Dez
                    {
                        text1[r1++] = int(uc / 100) + 48;
                        text1[r1++] = int((uc % 100) / 10) + 48;
                        text1[r1++] = int(uc % 10) + 48;
                        text1[r1++] = ' ';
                    }
                default: // If Ascii
                    break;
            }

            if (uc < 32)
            {
                switch (uc)
                {
                    case 13:
                        {
                            text2[r2++] = '\\';
                            text2[r2++] = 'r';
                            break;
                        }
                    case 10:
                        {
                            text2[r2++] = '\\';
                            text2[r2++] = 'n';
                            break;
                        }
                    case 11:
                        {
                            text2[r2++] = '\\';
                            text2[r2++] = 't';
                            break;
                        }
                    default:
                        {
                            text2[r2++] = '$';
                            text2[r2++] = '(';
                            if (int(data[n]) > 9)
                            {
                                text2[r2++] = char(int(uc / 10) + 48);
                            }
                            text2[r2++] = char(int(uc % 10) + 48);
                            text2[r2++] = ')';
                        }
                }
            }
            else
            {
                text2[r2++] = data[n];
            }
        }

        text1.resize(r1);
        if (displayType == 0)
        {
            text1 = "";
        }
        else if (text1.length() == 0)
        {
            text1 = "---";
        }
        else
        {
            text1 = prefix + text1;
        }

        text2.resize(r2);
        if (text2.length() == 0 && displayType == 0)
        {
            text2 = "---";
        }
        else if (text2.length() == 0)
        {
            text2 = "";
        }
        else if (displayType != 0)
        {
            text2 = "| " + text2;
        }

        text3.resize(10);
        if (data.length() == 0)
        {
            text3 = "";
        }
        else
        {
            text3 = " [" + QByteArray(QString::number(data.length()).toLatin1().data()) + "]";
        }

        if (!(ui.CheckBox->isChecked() && data.isEmpty() && endline.isEmpty()))
        {
            ui.textTransfer->append((QString)InOutChar + " " + text1 + text2 + text3);
        }
    }
 }

 //-------------------------------------------------------------------------------------------------------------------------------------------------
