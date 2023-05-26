#ifndef ALGOCANCELWIDGET_H
#define ALGOCANCELWIDGET_H

#include <QtGui>
#include <qwidget.h>
#include <qsharedpointer.h>
#include <qvector.h>
#include <qfuture.h>
#include <QtConcurrent/qtconcurrentrun.h>
#include <qmessagebox.h>

#include "ui_AlgoCancelWidget.h"

#include "common/functionCancellationAndObserver.h"
#include "common/param.h"
#include "common/retVal.h"



class AlgoCancelWidget : public QWidget
{
    Q_OBJECT

public:
    typedef ito::RetVal(*t_filterExt)  (QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, QSharedPointer<ito::FunctionCancellationAndObserver> observer);

    AlgoCancelWidget(t_filterExt filterFunc, QWidget *parent = NULL) :
        QWidget(parent),
        m_filterFunc(filterFunc)
    {
        ui.setupUi(this);

        ui.btnCancel->setVisible(false);
        ui.progressBar->setVisible(false);
        ui.lblProgress->setVisible(false);

        connect(&m_filterCallWatcher, SIGNAL(finished()), this, SLOT(filterCallFinished()));
    }

    ~AlgoCancelWidget() {}

protected:

    Ui::AlgoCancelWidget ui;

    QFuture<ito::RetVal> m_filterCall;
    QFutureWatcher<ito::RetVal> m_filterCallWatcher;

    t_filterExt m_filterFunc;

private slots:

    void on_btnStart_clicked()
    {
        ui.btnCancel->setVisible(true);
        ui.progressBar->setVisible(true);
        ui.lblProgress->setVisible(true);
        ui.progressBar->setMinimum(0);
        ui.progressBar->setMaximum(100);
        ui.progressBar->setValue(0);
        ui.btnStart->setVisible(false);

        QSharedPointer<ito::FunctionCancellationAndObserver> observer(new ito::FunctionCancellationAndObserver());

        //connect signals from the observer with the progressBar and label of the widget
        connect(observer.data(), SIGNAL(progressValueChanged(int)), ui.progressBar, SLOT(setValue(int)));
        connect(observer.data(), SIGNAL(progressTextChanged(QString)), ui.lblProgress, SLOT(setText(QString)));

        //connect the cancel button of the widget with the requestCancellation slot of the observer
        connect(ui.btnCancel, SIGNAL(clicked()), observer.data(), SLOT(requestCancellation()));

        QVector<ito::ParamBase> paramsMand, paramsOpt, paramsOut;

        //start the asychronous call. filterCallFinished will be called if the m_filterFunc method finished (either because it was finished or because the observer has been cancelled)
        //the call must be asychronous, else, the cancel button would not be executable
        m_filterCall = QtConcurrent::run(m_filterFunc, &paramsMand, &paramsOpt, &paramsOut, observer);
        m_filterCallWatcher.setFuture(m_filterCall);
    }




    //------------------------------------------------------------------------------------------------------------
    void filterCallFinished()
    {
        ito::RetVal retValue = m_filterCall.result();

        if (retValue.containsError())
        {
            QMessageBox::critical(this, "error", retValue.errorMessage());
        }
        else if (retValue.containsWarning())
        {
            QMessageBox::warning(this, "warning", retValue.errorMessage());
        }

        ui.btnCancel->setVisible(false);
        ui.progressBar->setVisible(false);
        ui.lblProgress->setVisible(false);
        ui.btnStart->setVisible(true);
    }


};

#endif
