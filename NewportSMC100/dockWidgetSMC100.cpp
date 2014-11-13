/* ********************************************************************
    Plugin "Newport SMC100" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut für Technische Optik (ITO),
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

#include "dockWidgetSMC100.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>
//#include <qsignalmapper.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetSMC100::DockWidgetSMC100(int uniqueID, ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator)
{
    ui.setupUi(this); 
    firstRun = true;
    m_stepSize = 0.5;

    m_pIncSignalMapper    = new QSignalMapper(this);
    m_pDecSignalMapper    = new QSignalMapper(this);
    m_pGoSignalMapper     = new QSignalMapper(this);
    m_pAbsPosSignalMapper = new QSignalMapper(this);
    //ui.lblID->setText(QString::number(uniqueID));

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::createUiListEntry(const int i)
{
    // Create outer elements
    QFrame *frame = new QFrame(ui.scrollAreaWidgetContents);
    ui.scrollAreaWidgetContents->layout()->setContentsMargins(0,0,0,0);
    QHBoxLayout *layout = new QHBoxLayout(frame);
    layout->setContentsMargins(0,0,0,0);
    frame->setLayout(layout);
     
    // Create innner elements and set option
    QLabel *nrLabel = new QLabel(QString::number(i),frame);
    QPushButton *incBtn = new QPushButton("+", frame);
    incBtn->setMaximumWidth(25);
    QPushButton *decBtn = new QPushButton("-", frame);
    decBtn->setMaximumWidth(25);
    QDoubleSpinBox *currSpin = new QDoubleSpinBox;
    currSpin->setDecimals(5);
    currSpin->setSuffix(" mm");
    currSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    currSpin->setDisabled(true);
    QDoubleSpinBox *destSpin = new QDoubleSpinBox;
    destSpin->setDecimals(5);
    destSpin->setSuffix(" mm");
    QPushButton *goBtn = new QPushButton(QIcon(),"", frame);

    // inser elements in Layout
    layout->insertWidget(0, nrLabel);
    layout->insertWidget(1, incBtn);
    layout->insertWidget(2, decBtn);
    layout->insertWidget(3, currSpin);
    layout->insertWidget(4, destSpin);
    layout->insertWidget(5, goBtn);
    frame->setLayout(layout);
    ui.verticalListLayout->insertWidget(ui.verticalListLayout->count()-1, frame);

    // Map signals from each line to corresponding slot
    connect(incBtn, SIGNAL(clicked()), m_pIncSignalMapper, SLOT(map()));
    m_pIncSignalMapper->setMapping(incBtn, i);
    connect(decBtn, SIGNAL(clicked()), m_pDecSignalMapper, SLOT(map()));
    m_pDecSignalMapper->setMapping(decBtn, i);
    connect(goBtn, SIGNAL(clicked()), m_pGoSignalMapper, SLOT(map()));
    m_pGoSignalMapper->setMapping(goBtn, i);
    connect(destSpin, SIGNAL(editingFinished()), m_pAbsPosSignalMapper, SLOT(map()));
    m_pAbsPosSignalMapper->setMapping(destSpin, i);
    
    // store Pointer to each spin box in a qvector for later occuring use
    m_pDestSpinBoxes.append(destSpin);
    m_pCurrSpinBoxes.append(currSpin); 

    m_absPosTarget.append(0);

    m_pListElements.append(frame);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::parametersChanged(QMap<QString, ito::Param> params)
{
    if (firstRun)
    {
        int nrOfAxis = params["nrOfAxis"].getVal<int>();
        ui.labelNrOfAxis->setNum(nrOfAxis);
        for (int i = 0; i < nrOfAxis; ++i)
        {
            // Create list entries
            createUiListEntry(i);
        }
        // Map all the signals to their destination
        connect(m_pIncSignalMapper, SIGNAL(mapped(const int &)), this, SLOT(incBtnClicked(const int &)));
        connect(m_pDecSignalMapper, SIGNAL(mapped(const int &)), this, SLOT(decBtnClicked(const int &)));
        connect(m_pGoSignalMapper, SIGNAL(mapped(const int &)), this, SLOT(goBtnClicked(const int &)));
        connect(m_pAbsPosSignalMapper, SIGNAL(mapped(const int &)), this, SLOT(absDestPosChanged(const int &)));

        // Don´t enter this part again
        firstRun = false; 
    }
    else
    {

    }
   /* ui.lblDevice1->setText( params["ctrlType"].getVal<char*>() );
    ui.lblDevice2->setText( params["ctrlName"].getVal<char*>() );
    ui.lblPiezo->setText( params["piezoName"].getVal<char*>() );
    bool hasMode = params["hasLocalRemote"].getVal<int>() > 0;
    ui.groupBoxMode->setVisible(hasMode);
    if (params["local"].getVal<int>() > 0)
    {
        ui.radioLocal->setChecked(true);
    }
    else
    {
        ui.radioRemote->setChecked(true);
    }*/
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::incBtnClicked(const int & i)
{
    setActuatorPosition(i, m_stepSize, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::decBtnClicked(const int & i)
{
    setActuatorPosition(i, -m_stepSize, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::goBtnClicked(const int & i)
{
    m_absPosTarget[i] = m_pDestSpinBoxes.at(i)->value();
    setActuatorPosition(i, m_absPosTarget[i], false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::absDestPosChanged(const int & i)
{
    if (m_pDestSpinBoxes.at(i)->hasFocus())
    {
        goBtnClicked(i);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
{
    if (m_pCurrSpinBoxes.size() == status.size() && m_pCurrSpinBoxes.size() == actPosition.size())
    {
        int i = 0;
        foreach(QDoubleSpinBox *cSB, m_pCurrSpinBoxes)
        {            
            cSB->setValue(actPosition[i]);

            if (actPosition.size() > 0)
            {
                cSB->setValue(actPosition[i]);
            }

            bool running = false;
            QString style;

            if (status[i] & ito::actuatorMoving)
            {
                style = "background-color: yellow";
                running = true;
            }
            else if (status[i] & ito::actuatorInterrupted)
            {
                style = "background-color: red";
            }
            else if (status[i] & ito::actuatorTimeout)
            {
                style = "background-color: #FFA3FD";
            }
            else
            {
                style = "background-color: ";
            }
            cSB->setStyleSheet(style);

            enableWidget(!running);
            ++i;
        }
    }
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::targetChanged(QVector<double> targetPositions)
{
    if (m_pDestSpinBoxes.size()  == targetPositions.size())
    {
        int i = 0;
        foreach(QDoubleSpinBox *dSB, m_pDestSpinBoxes)
        {            
            dSB->setValue(targetPositions[i]);

            if (targetPositions.size() > 0)
            {
                dSB->setValue(targetPositions[i]);
            }
            ++i;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::enableWidget(bool enabled)
{
    //ui.spinBoxTargetPos->setEnabled(enabled);
    //ui.btnUp->setEnabled(enabled);
    //ui.btnDown->setEnabled(enabled);
    //ui.groupBoxMode->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::on_btnStart_clicked()
{
    //setActuatorPosition(0, ui.spinBoxTargetPos->value() / 1000.0, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSMC100::on_btnRefresh_clicked()
{
    //requestActuatorStatusAndPositions(true, true);
}
