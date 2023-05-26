/* ********************************************************************
    Plugin "FittingFilters" for itom software
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

#include "fittingfilters.h"
#include "DataObject/dataObjectFuncs.h"
#include <qvariant.h>
#include <qnumeric.h>

//#include "common/helperCommon.h"

using namespace ito;

//---------------------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::fitPolynom2DDoc = QObject::tr("Fit a polynomial p(x,y) of order (orderX, orderY) in x- and y-direction. \n\
\n\
The fit function always looks like this: \n\
\n\
f(x,y) = sum_{m=0}^{M} sum_{n=0}^{N} ( p_nm * x^n * y^m ) \n\
\n\
This definition is slightly different from the polynomial fitted by the similar function 'polyfitWeighted2d'. \n\
\n\
Puts the fitted points into the data object 'fittedImage'. This method does not weight the input values and does not \n\
return the coefficients for the polynomial. Use polyfitWeighted2d if you want to have an enhanced fit.");

ito::RetVal FittingFilters::fitPolynom2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = retOk;
    ito::Param param;

    if (!paramsMand || !paramsOpt || !paramsOut)
    {
        return ito::RetVal(ito::retError, 0, tr("uninitialized vector for mandatory, optional or output parameters!").toLatin1().data());
    }

    paramsMand->clear();
    paramsOpt->clear();
    paramsOut->clear();

    param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object").toLatin1().data());
    paramsMand->append(param);

    param = Param("fittedImage", ParamBase::DObjPtr | ParamBase::In | ParamBase::Out , NULL, tr("destination data object with fitted values").toLatin1().data());
    paramsMand->append(param);

    param = Param("orderX", ParamBase::Int, NULL, tr("order of the fitting polynomial in x-direction").toLatin1().data());
    paramsMand->append(param);

    param = Param("orderY", ParamBase::Int, NULL, tr("order of the fitting polynomial y-direction").toLatin1().data());
    paramsMand->append(param);

    param = Param("replaceNaN", ParamBase::Int, 0, 1, 0, tr("if 0 infinite values in input image will be copied to output").toLatin1().data());
    paramsOpt->append(param);

    *paramsOut << Param("sigma", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Variance value *sigma* of polynomial fit.").toLatin1().data());

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFilters::fitPolynom2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjImages = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    ito::DataObject *dObjDst    = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>() );

    int gradX = static_cast<int>( (*paramsMand)[2].getVal<int>() );
    int gradY = static_cast<int>( (*paramsMand)[3].getVal<int>() );

    int fillNaNValues = static_cast<int>( (*paramsOpt)[0].getVal<int>() );

    retval = ito::dObjHelper::verify2DDataObject(dObjImages, "sourceImage", gradY + 1, std::numeric_limits<short>::max(),
                                                                            gradX + 1, std::numeric_limits<short>::max(),
                                                                            8,
                                                                            tUInt8 , tInt8 , tUInt16 , tInt16 , tUInt32 , tInt32 , tFloat32 , tFloat64);

    if (retval.containsError())
    {
        if (dObjImages != NULL && dObjImages->getDims() == 3)
        {
            retval = ito::dObjHelper::verify3DDataObject(dObjImages, "sourceImage", 1, 1,
                                                                                    gradY + 1, std::numeric_limits<short>::max(),
                                                                                    gradX + 1, std::numeric_limits<short>::max(),
                                                                                    8,
                                                                                    tUInt8 , tInt8 , tUInt16 , tInt16 , tUInt32 , tInt32 , tFloat32 , tFloat64);
        }
        if (retval.containsError())
        {
            return retval;
        }
    }

    if (dObjDst == NULL)
    {
        return ito::RetVal(retError, 0, tr("destination matrix is NULL").toLatin1().data());
    }

    cv::Mat *plane = reinterpret_cast<cv::Mat*>(dObjImages->get_mdata()[ dObjImages->seekMat(0) ]);
    cv::Mat inputImage;

    if (plane->type() != CV_64FC1)
    {
        plane->convertTo(inputImage, CV_64FC1);
    }
    else
    {
        inputImage = *plane;
    }

    cv::Mat outputImage = cv::Mat(plane->rows, plane->cols, CV_64FC1);

    int  *x,*y;
    int   xsize,ysize,i;
    struct Koeffizienten koeff;
    double Sigma = 0.0;

    xsize=plane->cols;
    ysize=plane->rows;
    x = new int[xsize];
    y = new int[ysize];

    for (i=0;i<xsize;i++)
    {
        x[i] = i;
    }

    for (i=0;i<ysize;i++)
    {
        y[i] = i;
    }

    retval += polyfit(x, y, &inputImage, &outputImage, gradX, gradY, xsize, ysize, &Sigma, &koeff, fillNaNValues > 0);

    *dObjDst = ito::DataObject(dObjImages->getDims(), dObjImages->getSize(), ito::tFloat64, &outputImage, 1);

    dObjImages->copyAxisTagsTo(*dObjDst);
    dObjImages->copyTagMapTo(*dObjDst);

        // Add Protokoll
//    char prot[81] = {0};
//    _snprintf(prot, 80, "2D polynomical fit with order x = %i and y = %i", gradX, gradY);
//    dObjDst->addToProtocol(std::string(prot));
    QString msg;
    msg = tr("2D polynomical fit with order x = %1 and y = %2").arg(gradX).arg(gradY);
    dObjDst->addToProtocol(std::string(msg.toLatin1().data()));

    delete[] x;
    delete[] y;

    if (retval == retOk)
    {
        (*paramsOut)[0].setVal<double>(Sigma);
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/* ******************** Polyfit.C   23.3.93 ***************************** */
/* ***********************************************************************
    Mit dieser Funktion werden Polynomkoeffizienten einer Ausgleichsflaeche
    durch die Messpunkte berechnet. Sie gibt folgende Fehlercodes zurrueck:

    0:  Fehlerfrei beendet
    1:  Fehler bei den Feldgrenzen
    2:  gradX und gradY =0, Funktion abgebrochen
    3:  gradX und/oder gradY zu gross
    4:  Fehler bei Speicherzuweisung in Berechne_Koeff
    5:  Fehler bei Speicherzuweisung fuer Rekursionskoeffizienten
    6:  Fehler bei Speicherzuweisung fuer Werte, NormX, NormY oder Sum
    7:  Fehler bei Speicherzuweisung fuer ZeilenSumme
    8:  Fehler bei berechnung der Fehlerquadrate

    Beim Aufruf muessen uebergeben werden:

    x         - Ein Zeiger auf das Array, das die X-Koordinaten der Mess-
              punkte enthaelt.
    y         - Ein Zeiger auf das Array, das die Y-Koordinaten der Mess-
              punkte enthaelt.
    Messwerte - Ein Zeiger (float) auf den Anfang des Arrays, in dem die
              Messwerte zu finden sind. Es muss von der Form M[x][y] sein.
    gradX,    - die Polynomgrade der Fitfunktion in X/Y - Richtung.
    gradY
    AnzahlX,  - die Zahl der Messpunkte in X/Y - Richtung.
    AnzahlY
    Sigma     - Ein Pointer auf eine float - Variable, in der die Fehler-
              quadrat Summe uebergeben wird.

 Die Funktion uebergibt dann die Polynomkoeffizienten und alle anderen Infos
 zur Berechnung der Funktionswerte an die Structure koeff.
 ********************************************************************** */
ito::RetVal FittingFilters::polyfit(int *x, int *y, cv::Mat *dblData, cv::Mat *dblFittedData, int gradX, int gradY, int sizeX, int sizeY, double *sigma, struct Koeffizienten *koeff, bool fillNaNValues)
//
//static int Polyfit(float x[], float y[],
//           DataObject *data, DataObject *fit,
//           int gradX, int gradY, int AnzahlX, int AnzahlY,
//           double *Sigma, struct Koeffizienten *koeff)
{
    RetVal retValue(retOk);
    int   i, j, n, m;            // Zaehlvariable
    int maxGrad;             // Maximal vorkommender Grad

    double   tx, ty,           // x, y Parameter
    dx, dy,             // Schrittweiten der X/Y-Koordinaten
    z;             // berechnete Z-Koordinate
    float64 buf;
    double    *Sum=NULL,            // Zwischen- Koeffizienten
    *NormX=NULL,        // Nenner der Koeffizienten
    *NormY=NULL,
    *Werte=NULL,        // Pn-Koeffizienten
    *ZeilenSumme=NULL;    // Zaehler der Koeffizienten

    const float64 *lineBuf = NULL;
    float64 *lineBufOutput = NULL;
    double inf = std::numeric_limits<double>::infinity();
    double nan =std::numeric_limits<double>::quiet_NaN();

    // Kontrolle der Polynomgrade -------------------------------------------
    if (gradX < 1 || gradY < 1)
    {
        return retValue += ito::RetVal(retError, 2, tr("2:  polynomial orders must not be below 1, aborted").toLatin1().data());
    }

    if ( (gradX > (sizeX - 1)) || (gradY > (sizeY - 1)) )
    {
        return retValue += ito::RetVal(retError, 3, tr("3:  polynomial order in x and/or y too big, aborted").toLatin1().data());
    }

    maxGrad = gradX < gradY ? gradY : gradX;

    // Uebergabe der Grade und Anzahlen an koeff ------------------------------
    koeff->gradX=gradX;
    koeff->gradY=gradY;
    koeff->sizeX=sizeX;
    koeff->sizeY=sizeY;

    // Initialisierung der Rekursionskoeffizienten --------------------------
    koeff->alphaX = NULL;
    koeff->alphaY = NULL;
    koeff->betaX = NULL;
    koeff->betaY = NULL;
    koeff->gammaX = NULL;
    koeff->gammaY = NULL;
    Sum = NULL;
    koeff->b = NULL;
    ZeilenSumme = NULL;

    koeff->alphaX= (double *)calloc(koeff->gradX+1, sizeof(double));
    koeff->alphaY= (double *)calloc(koeff->gradY+1, sizeof(double));
    koeff->betaX= (double *)calloc(koeff->gradX+1, sizeof(double));
    koeff->betaY= (double *)calloc(koeff->gradY+1, sizeof(double));
    koeff->gammaX= (double *)calloc(koeff->gradX+1, sizeof(double));
    koeff->gammaY= (double *)calloc(koeff->gradY+1, sizeof(double));

    if (koeff->alphaY == NULL || koeff->alphaX == NULL || koeff->betaX == NULL || koeff->betaY == NULL || koeff->gammaX == NULL || koeff->gammaY == NULL)
    {
        retValue += ito::RetVal(retError, 5, tr("5:  error allocating memory for recursion coefficients").toLatin1().data());
        goto Error;
    }

    // Berechnung der Rekursionskoeffizienten --------------------------------
    retValue += calcKoeff(koeff->sizeX, koeff->gradX, koeff->alphaX, koeff->betaX, koeff->gammaX);
    if (retValue.containsError())
    {
        goto Error;
    }

    retValue += calcKoeff(koeff->sizeY, koeff->gradY, koeff->alphaY, koeff->betaY, koeff->gammaY);
    if (retValue.containsError())
    {
        goto Error;
    }

    // Initialisierung von Werte, NormX, NormY, Sum --------------------------
    Sum=(double *)calloc((koeff->gradX+1)*(koeff->gradY+1), sizeof(double));
    koeff->b=(double *)calloc((koeff->gradX+1)*(koeff->gradY+1), sizeof(double));
    Werte=(double *)calloc(maxGrad+1, sizeof(double));
    NormX=(double *)calloc(koeff->gradX+1, sizeof(double));
    NormY=(double *)calloc(koeff->gradY+1, sizeof(double));

    if (Sum == NULL || koeff->b == NULL || Werte == NULL || NormX == NULL || NormY == NULL)
    {
        retValue += RetVal(retError, 6, tr("6: error allocating memory for NormX, NormY or Sum").toLatin1().data());
        goto Error;
    }

    // Berechnung der Schrittweiten ------------------------------------------
    dx = 1;
    dy = 1;

    // Berechnung der NormX mit der Funktion OrthPolAuswerten ----------------
    for (n=0; n < koeff->sizeX; n++)
    {
        if (n == 0)
        {
            tx=0;
        }
        else
        {
            tx=(x[n]-x[0])/dx;
        }

        OrthPolAuswerten(koeff->sizeX, koeff->gradX, tx, Werte, koeff->alphaX, koeff->betaX, koeff->gammaX);

        for (i=0 ; i <= koeff->gradX ; i++)
        {
            NormX[i]+=Werte[i]*Werte[i];
        }
    }

    // Berechnen von NormX und der Zeilensumme  ------------------------------
    ZeilenSumme=(double *)calloc(koeff->gradX+1, sizeof(double));
    if (ZeilenSumme == NULL)
    {
        retValue += ito::RetVal(retError, 7, tr("7:  error allocating memory for line sum").toLatin1().data());
        goto Error;
    }

    for (m=0 ; m < koeff->sizeY ; m++)
    {
        // Initialisierung der Zeilensumme ------------------------------------
        memset(ZeilenSumme, 0, (koeff->gradX+1) * sizeof(double));
        lineBuf = dblData->ptr< double >(m);

        // Berechnung der Zeilensumme  ----------------------------------------
        for (n= 0 ; n < koeff->sizeX ; n++)
        {
            if (n == 0)
            {
                tx=0;
            }
            else
            {
                tx=(x[n]-x[0]); /*/dx;*/ //dx=1
            }

            OrthPolAuswerten(koeff->sizeX, koeff->gradX, tx, Werte, koeff->alphaX, koeff->betaX, koeff->gammaX);

            buf = lineBuf[n];
            //nicht so tolle version (verfaelscht das ergebnis durch addition von 0.0 bei nan-werten - kommentar marc gronle, interpolation waere besser, aber aufwand)
            if (qIsFinite(buf))
            {
                for (i=0 ; i <= koeff->gradX ; i++)
                {
                    ZeilenSumme[i]+=Werte[i]*buf;
                }
            }
            else
            {
                for (i=0 ; i <= koeff->gradX ; i++)
                {
                    ZeilenSumme[i]+=0.0;
                }
            }
        }

        // Berechnung von NormY  ----------------------------------------------
        if (m == 0)
        {
            ty=0;
        }
        else
        {
            ty=(y[m]-y[0]); /*/dy;*/ //dy=1
        }

        OrthPolAuswerten(koeff->sizeY, koeff->gradY, ty, Werte, koeff->alphaY, koeff->betaY, koeff->gammaY);

        for (j=0 ; j <= koeff->gradY ; j++)
        {
            NormY[j]+=Werte[j]*Werte[j];
        }

        // Berechnung von Sum -------------------------------------------------
        for (i=0 ; i <= koeff->gradX ; i++)
        {
            for (j=0 ; j <= koeff->gradY ; j++)
            {
                Sum[i*(koeff->gradY+1)+j]+=ZeilenSumme[i]*Werte[j];
            }
        }
    }

    // Weitere Berechnungen mit Sum ------------------------------------------
    for (i=0 ; i <= koeff->gradX ; i++)
    {
        for (j=0 ; j <= koeff->gradY ; j++)
        {
            Sum[i*(koeff->gradY+1)+j]/=NormX[i]*NormY[j];
        }
    }

    // Uebergabe der Koeffizienten an die Structure
    for (i=0 ; i <= koeff->gradX ; i++)
    {
        for (j=0 ; j <= koeff->gradY ; j++)
        {
            *(koeff->b + i*(koeff->gradY+1) + j)=Sum[ i*(koeff->gradY+1) + j];
        }
    }

    // Berechnung der Summe der Fehlerqudrate --------------------------------
    *sigma=0.0;

    if (fillNaNValues)
    {
        for (m=0 ; m < koeff->sizeY ; m++)
        {
            lineBuf = dblData->ptr< double >(m);
            lineBufOutput = dblFittedData->ptr< double >(m);
            ty=(y[m]-y[0])/dy;
            //STATUSBAR(1.0*n/koeff->AnzahlX);

            for (n=0 ; n < koeff->sizeX ; n++)
            {
                tx=(x[n]-x[0])/dx;
                z=Fitwerte(tx, ty, koeff);

                buf = lineBuf[n];

                if ( qIsFinite(buf))
                {
                    lineBufOutput[n] = z;
                    *sigma+=(buf-z)*(buf-z);
                }
                else
                {
                    lineBufOutput[n] = z;
                    //*sigma += 0.0;
                }
            }
        }
    }
    else
    {
        for (m = 0 ; m < koeff->sizeY ; m++)
        {
            lineBuf = dblData->ptr< double >(m);
            lineBufOutput = dblFittedData->ptr< double >(m);
            ty=(y[m]-y[0])/dy;
            //STATUSBAR(1.0*n/koeff->AnzahlX);

            for (n = 0 ; n < koeff->sizeX ; n++)
            {
                tx=(x[n]-x[0])/dx;
                z=Fitwerte(tx, ty, koeff);

                buf = lineBuf[n];

                if ( qIsFinite(buf))
                {
                    lineBufOutput[n] = z;
                    *sigma+=(buf-z)*(buf-z);
                }
                else
                {
                    lineBufOutput[n] = nan;
                }
             }
        }
    }

    //STATUSBAR(-1);

Error:
    if (koeff->alphaX) free(koeff->alphaX);
    if (koeff->alphaY) free(koeff->alphaY);
    if (koeff->betaX) free(koeff->betaX);
    if (koeff->betaY) free(koeff->betaY);
    if (koeff->gammaX) free(koeff->gammaX);
    if (koeff->gammaY) free(koeff->gammaY);
    if (Sum!=NULL)        free(Sum);
    if (koeff->b!=NULL)        free(koeff->b);
    if (NULL!=Werte)        free(Werte);
    if (NormX!=NULL)        free(NormX);
    if (NormY!=NULL)        free(NormY);
    if (NULL!=ZeilenSumme)    free(ZeilenSumme);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFilters::calcKoeff(int anzahl,int PolyGrad, double *Alpha, double *Beta, double *Gamma)
{
    int     i, j, k;
    double   *P, *S, *Q;

    // *** Berechnung der P[i][j] nach (11.10)  *****************************
    //P = S = Q = NULL;
    P= (double *)calloc((PolyGrad+1)*(PolyGrad+1), sizeof(double));
    S= (double *)calloc((PolyGrad+1)*(PolyGrad+1), sizeof(double));
    Q= (double *)calloc((PolyGrad+1)*(PolyGrad+1), sizeof(double));   // Initialisieren der Matrix Q

    if (P == NULL || S == NULL || Q == NULL)
    {
        if (P) free(P);
        if (S) free(S);
        if (Q) free(Q);
        return ito::RetVal(retError, 1000, tr("error while allocating memory").toLatin1().data());
    }

    for ( i=0 ; i<=PolyGrad ; i++)  // laeuft von 0 bis Grad +1
    {
        *(P + i*(PolyGrad+1) )=1;      // P[i][0] = 1
        for (j=1; j<=PolyGrad; j++)
        {
            // P[i][j] = -P[i][j-1]/j*(i-j+1)*(i+j)/j/(anzahl-j);
            *(P + i*(PolyGrad+1) + j)=-*(P + i*(PolyGrad+1) + j-1)/j*(i-j+1)*(i+j)/j/(anzahl-j);
        }
    }

    // *** Berechnung der Stirlingzahlen (11.13) ****************************

    for (i=0; i<=PolyGrad; i++)
    {
        *(S + i*(PolyGrad+1) ) = 0;
        *(S + i*(PolyGrad+1) + i) = 1;

        for (j=i+1; j<=PolyGrad; j++)
        {
            *(S + i*(PolyGrad+1) + j) = 0;
        }
    }

    for (i=2; i<=PolyGrad; i++)
    {
        *(S + i*(PolyGrad+1) + 1) = -(i-1)* *(S + (i-1)*(PolyGrad+1) +1); // Rekursionsformel
    }

    for (i=3; i<=PolyGrad; i++)   // fuer die Stirlingzahlen
    {
        for (j=2; j<=i-1; j++)
        {
            *(S + i*(PolyGrad+1) + j) = *(S + (i-1)*(PolyGrad+1) + j-1)-(i-1)* *(S + (i-1)*(PolyGrad+1) + j);
        }
    }

    /**** Berechnung der Polynom-Koeffizienten ****
             n              n-1
    P (t) = Q[n][n] t  + Q[n][n-1] t    + ... + Q[n,0]
     n
    ************************************************/

    // Matrixmultiplikation
    for (i=0; i<=PolyGrad; i++)
    {
        for (j=0; j<=i; j++)
        {
            for (k=j; k<=i; k++)    // von P und S nach Q
            {
                *(Q + i*(PolyGrad+1) + j) += *(P + i*(PolyGrad+1) + k) * *(S + k*(PolyGrad+1) + j);
            }
        }
    }

    free(P); P=NULL;
    free(S); S=NULL;

    /*************************************************
     Berechnung der Koeffizienten fuer die Rekursion. Dass so eine Rekursion
     existiert, ist klar, es genuegt alpha, beta und gamma so zu bestimmen, dass
     die ersten drei Koeffizienten des Polynoms Pn richtig dargestellt werden,
     die anderen sind dann automatisch richtig.
    ***************************************************/
    for (i = 2; i<=PolyGrad; i++)
    {
        if ( ( *(Q + (i-1)*(PolyGrad+1) + i-1) != 0) && ( *(Q + (i-2)*(PolyGrad+1) + i-2) != 0))
        {
            Alpha[i] = *(Q + i*(PolyGrad+1) + i) / *(Q + (i-1)*(PolyGrad+1) + i-1);
            Beta[i] = ( *(Q + i*(PolyGrad+1) + i-1) - Alpha[i] *
               *(Q + (i-1)*(PolyGrad+1) + i-2) ) / *(Q + (i-1)*(PolyGrad+1) + i-1);

            if (i > 2)
                Gamma[i] = ( *(Q + i*(PolyGrad+1) + i-2) - Alpha[i] * *(Q + (i-1)*(PolyGrad+1) + i-3)
                        - Beta[i]  * *(Q + (i-1)*(PolyGrad+1) + i-2) ) /
                        *(Q + (i-2)*(PolyGrad+1) + i-2);
            else
                Gamma[i] = ( *(Q + i*(PolyGrad+1) + i-2) - Beta[i] *
                        *(Q + (i-1)*(PolyGrad+1) + i-2) ) / *(Q + (i-2)*(PolyGrad+1) + i-2);
        }
        else
        {
            break;
        }
    }

    free(Q);Q=NULL;

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void FittingFilters::OrthPolAuswerten(int anzahl, int PolyGrad, double t, double *W, double *alpha,double *beta, double *gamma)
{
    int i;
    W[0] = 1.0;

    if (PolyGrad > 0)
    {
        W[1] = 1.0 - 2.0 * t / (anzahl - 1);
    }

    for (i = 2; i <= PolyGrad; i++)
    {
        W[i] = (alpha[i] * t + beta[i]) * W[i - 1] + gamma[i] * W[i - 2];
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
double FittingFilters::Fitwerte(double tx, double ty, struct Koeffizienten *koeff)
{
    int      n, m;        // Zaehlvariable
    double    z,           // Ergebniszwischenspeicher
          *Px, *Py;    // berechnete orthogonale Koeffizienten

    // ----------------- Allgemeine Initialisierungen -----------------------
    Px=(double *)calloc(koeff->gradX+1, sizeof(double));
    Py=(double *)calloc(koeff->gradY+1, sizeof(double));

    // ---------------- Berechnung der P-Koeffizienten -----------------------
    OrthPolAuswerten(koeff->sizeX, koeff->gradX, tx, Px, koeff->alphaX, koeff->betaX, koeff->gammaX);
    OrthPolAuswerten(koeff->sizeY, koeff->gradY, ty, Py, koeff->alphaY, koeff->betaY, koeff->gammaY);

    // --------- Berechnung des Funktionswertes nach Gl. 11.8 ----------------
    for (n=0, z=0.0 ; n <= koeff->gradX ; n++)
    {
        for (m=0 ; m <= koeff->gradY ; m++)
        {
            z+=koeff->b[n*(koeff->gradY+1)+m] * Px[n] * Py[m];
        }
    }

    free(Px);
    free(Py);

    return(z);
}
