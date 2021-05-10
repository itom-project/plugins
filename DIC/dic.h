/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
	URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIC_H
#define DIC_H

#include "common/addInInterface.h"
#include <qsharedpointer.h>
#include <qmap.h>
#include <DataObject/dataobj.h>

#if (USEOPENMP)
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
/**
*\class    DICInterface
*
*\brief    Interface-Class for DIC-Class
*
*\sa    AddInDataIO, DIC
*
*/
class DICInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        DICInterface();
        ~DICInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

/** @class DIC
*   @brief Algorithms used for digital image correlation
*
*   Descreve o plugin ...
*/
class DIC : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        DIC();
        ~DIC();

    public:
        friend class DICInterface;

        static const QString DICInitialGuessDoc;
        static ito::RetVal DICInitialGuess(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICInitialGuessParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICInterpolationDoc;
        static ito::RetVal DICInterpolation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICInterpolationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICDisplacementDoc;
        static ito::RetVal DICDisplacement(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICDisplacementParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICDisplacementFFDoc;
        static ito::RetVal DICDisplacementFF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICDisplacementFFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICGenerateImageDoc;
        static ito::RetVal DICGenerateImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICGenerateImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICGenerateProjImageDoc;
        static ito::RetVal DICGenerateProjImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICGenerateProjImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICDeformationDoc;
        static ito::RetVal DICDeformation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICDeformationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICSplineCoeffsDoc;
        static ito::RetVal DICSplineCoeffs(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICSplineCoeffsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString DICSplineValsDoc;
        static ito::RetVal DICSplineVals(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal DICSplineValsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

    private:
        std::vector<int> m_cudaDevices;
        int m_lastDevice;
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DIC_H
