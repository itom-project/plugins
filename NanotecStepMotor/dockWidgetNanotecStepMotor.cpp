/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "dockWidgetNanotecStepMotor.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetNanotecStepMotor::DockWidgetNanotecStepMotor(ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator)
{
    ui.setupUi(this);
    firstRun = true;

    m_pIncSignalMapper    = new QSignalMapper(this);
    m_pDecSignalMapper    = new QSignalMapper(this);
    m_pGoSignalMapper     = new QSignalMapper(this);
    m_pAbsPosSignalMapper = new QSignalMapper(this);
    //ui.lblID->setText(QString::number(uniqueID));

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetNanotecStepMotor::~DockWidgetNanotecStepMotor()
{
//    int i = 0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::createUiListEntry(const int i)
{
    // Create outer elements
    QFrame *frame = new QFrame(ui.scrollAreaWidgetContents);
    ui.scrollAreaWidgetContents->layout()->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout *layout = new QHBoxLayout(frame);
    layout->setContentsMargins(0, 0, 0, 0);
    frame->setLayout(layout);

    // Create innner elements and set option
    QLabel *nrLabel = new QLabel(QString::number(i), frame);
    nrLabel->setAlignment(Qt::AlignCenter);

    QPushButton *incBtn = new QPushButton("+", frame);

    QPushButton *decBtn = new QPushButton("-", frame);

    QDoubleSpinBox *currSpin = new QDoubleSpinBox;
    currSpin->setDecimals(5);
    currSpin->setSuffix(" mm");
    currSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    currSpin->setDisabled(true);
    currSpin->setMaximumHeight(20);
    currSpin->setMinimum(-std::numeric_limits<double>::max());
    currSpin->setMaximum(std::numeric_limits<double>::max());

    QDoubleSpinBox *destSpin = new QDoubleSpinBox;
    destSpin->setDecimals(5);
    destSpin->setSuffix(" mm");
    destSpin->setMaximumHeight(20);
    destSpin->setMinimum(-std::numeric_limits<double>::max());
    destSpin->setMaximum(std::numeric_limits<double>::max());

    QPushButton *goBtn = new QPushButton(QIcon(":/widget/run.png"), "", frame);
    goBtn->setMaximumWidth(20);
    goBtn->setMaximumHeight(20);
    goBtn->setMinimumWidth(8);

    // Set Width management
    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    nrLabel->setSizePolicy(sizePolicy);
    nrLabel->setMinimumWidth(20);
    nrLabel->setMaximumWidth(20);

    incBtn->setSizePolicy(sizePolicy);
    incBtn->setMinimumWidth(12);
    incBtn->setMinimumWidth(18);

    decBtn->setSizePolicy(sizePolicy);
    decBtn->setMinimumWidth(12);
    decBtn->setMinimumWidth(18);

    destSpin->setSizePolicy(sizePolicy);

    goBtn->setSizePolicy(sizePolicy);
    goBtn->setMinimumWidth(8);
    goBtn->setMinimumWidth(18);

    QSizePolicy sizePolicyEx(QSizePolicy::Expanding, QSizePolicy::Preferred);
    destSpin->setSizePolicy(sizePolicyEx);
    currSpin->setSizePolicy(sizePolicyEx);
    currSpin->setMaximumWidth(85);
    destSpin->setMaximumWidth(85);


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
    m_pIncButtons.append(incBtn);
    m_pDecButtons.append(decBtn);
    m_pGoButtons.append(goBtn);

    m_absPosTarget.append(0);

    m_pListElements.append(frame);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::parametersChanged(QMap<QString, ito::Param> params)
{
    if (firstRun)
    {
        int nrOfAxis = params["numaxis"].getVal<int>();
        ui.btnStart->setIcon(QIcon(":/widget/run_multi"));
        ui.labelNrOfAxis->setNum(nrOfAxis);
        ui.ctrlName->setText(params["deviceID"].getVal<char*>());
        QString sID = QString::number(params["devicePort"].getVal<int>());
        ui.ctrlIDLabel->setText(sID);
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
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::incBtnClicked(const int & i)
{
    setActuatorPosition(i, ui.spinStepSize->value(), true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::decBtnClicked(const int & i)
{
    setActuatorPosition(i, -ui.spinStepSize->value(), true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::goBtnClicked(const int & i)
{
    m_absPosTarget[i] = m_pDestSpinBoxes.at(i)->value();
    setActuatorPosition(i, m_absPosTarget[i], false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::absDestPosChanged(const int & i)
{
    if (m_pDestSpinBoxes.at(i)->hasFocus())
    {
        goBtnClicked(i);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
{
//    qDebug() << "dockWidet:actuatorStatusChanged:: " << status << actPosition;
    if (m_pCurrSpinBoxes.size() == status.size() && (m_pCurrSpinBoxes.size() == actPosition.size() || actPosition.size() == 0))
    {
        int i = 0;
        foreach(QDoubleSpinBox *cSB, m_pCurrSpinBoxes)
        {
            if (actPosition.size() > 0)
            {
                cSB->setValue(actPosition[i]);
            }

            bool running = false;
            QString style;

            if (status[i] & ito::actuatorMoving)
            {
                style = "background-color: yellow";
                m_pIncButtons[i]->setEnabled(false);
                m_pDecButtons[i]->setEnabled(false);
                m_pGoButtons[i]->setEnabled(false);
                running = true;
            }
			else if (status[i] & ito::actuatorInterrupted)
            {
                style = "background-color: red";
                m_pIncButtons[i]->setEnabled(true);
                m_pDecButtons[i]->setEnabled(true);
                m_pGoButtons[i]->setEnabled(true);
            }
            else if (status[i] & ito::actuatorTimeout)
            {
                style = "background-color: #FFA3FD";
                m_pIncButtons[i]->setEnabled(true);
                m_pDecButtons[i]->setEnabled(true);
                m_pGoButtons[i]->setEnabled(true);
            }
            else
            {
                style = "background-color: ";
                m_pIncButtons[i]->setEnabled(true);
                m_pDecButtons[i]->setEnabled(true);
                m_pGoButtons[i]->setEnabled(true);
            }
            cSB->setStyleSheet(style);

            enableWidget(!running);
            ++i;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::targetChanged(QVector<double> targetPositions)
{
//    qDebug() << targetPositions;

    if (m_pDestSpinBoxes.size() == targetPositions.size())
    {
        int i = 0;
        foreach(QDoubleSpinBox *dSB, m_pDestSpinBoxes)
        {
            dSB->setValue(targetPositions[i]);

            //if (targetPositions.size() > 0)
            //{
            //    dSB->setValue(targetPositions[i]);
            //}
            ++i;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::enableWidget(bool enabled)
{
/*    ui.spinBoxTargetPos->setEnabled(enabled);
    ui.btnUp->setEnabled(enabled);
    ui.btnDown->setEnabled(enabled);
    ui.btnRefresh->setEnabled(enabled);*/
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::on_btnStart_clicked()
{
    QVector<int> axis;
    QVector<double> values;
    for (int i = 0; i < m_pDestSpinBoxes.size(); ++i)
    {
        axis.append(i);
        values.append(m_pDestSpinBoxes.at(i)->value());
    }
    setActuatorPosition(axis, values, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::on_btnRefresh_clicked()
{
    requestActuatorStatusAndPositions(true, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNanotecStepMotor::on_btnCancel_clicked()
{
    setActuatorInterrupt();
}
