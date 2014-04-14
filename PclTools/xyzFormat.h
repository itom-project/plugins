#ifndef XYZFORMAT_H
#define XYZFORMAT_H

#include "common/helperCommon.h"
#include "PointCloud/pclStructures.h"

#include <qtextstream.h>


template<typename _Tp> ito::RetVal writeXYZ(const pcl::PointCloud<_Tp>& pointCloud, QTextStream &stream)
{
    ito::RetVal retval;

    stream.setRealNumberNotation( QTextStream::FixedNotation );
    stream.setRealNumberPrecision( 6 );

    std::vector<_Tp, Eigen::aligned_allocator<_Tp> > points;

    points = pointCloud.points;
    
    for (size_t i = 0; i < pointCloud.points.size (); ++i)
    {
      if (!pcl_isfinite (pointCloud.points[i].x) || 
          !pcl_isfinite (pointCloud.points[i].y) || 
          !pcl_isfinite (pointCloud.points[i].z))
        continue;

      stream << pointCloud.points[i].x << " " << pointCloud.points[i].y << " " << pointCloud.points[i].z << endl;
    }

    return retval;
}

template<typename _Tp> ito::RetVal readXYZ(QTextStream &stream, pcl::PointCloud<_Tp>& pointCloud)
{
    ito::RetVal retval;

    stream.setRealNumberNotation( QTextStream::FixedNotation );
    stream.setRealNumberPrecision( 6 );

    size_t guessedSize = 0;
    size_t realSize = 0;
    QString currentLine;
    QStringList elems;
    float x,y,z;
    bool okx, oky, okz;

    while( !stream.atEnd() )
    {
        currentLine = stream.readLine();
        elems = currentLine.split(" ");

        if(elems.size() >= 3)
        {
            x = elems[0].toFloat(&okx);
            y = elems[1].toFloat(&oky);
            z = elems[2].toFloat(&okz);

            if(okx && oky && okz)
            {
                //point is ok
                if(realSize >= guessedSize)
                {
                    guessedSize += 500;
                    pointCloud.points.resize( guessedSize );
                }

                pointCloud.points[realSize].x = x;
                pointCloud.points[realSize].y = y;
                pointCloud.points[realSize].z = z;

                realSize ++;
            }
        }
    }

    pointCloud.resize( realSize );


    return retval;
}

#endif