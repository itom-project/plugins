/* ********************************************************************
    Plugin "PCLTools" for itom software
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

#ifndef PCL_FILTERS_RANDOM_SUBSAMPLE_CORRECTED_H_
#define PCL_FILTERS_RANDOM_SUBSAMPLE_CORRECTED_H_

#ifndef Q_MOC_RUN
#include <pcl/filters/filter_indices.h>
#endif
#include <time.h>
#include <limits.h>



  template<typename PointT>
  class RandomSampleCorrected : public pcl::FilterIndices<PointT>
  {
    using pcl::FilterIndices<PointT>::filter_name_;
    using pcl::FilterIndices<PointT>::getClassName;
    using pcl::FilterIndices<PointT>::indices_;
    using pcl::FilterIndices<PointT>::input_;

    typedef typename pcl::FilterIndices<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      RandomSampleCorrected () : sample_ (UINT_MAX), seed_ (static_cast<unsigned int> (time (NULL)))
      {
        filter_name_ = "RandomSample";
      }

      /** \brief Set number of indices to be sampled.
        * \param sample
        */
      inline void
      setSample (unsigned int sample)
      {
        sample_ = sample;
      }

      /** \brief Get the value of the internal \a sample parameter.
        */
      inline unsigned int
      getSample ()
      {
        return (sample_);
      }

      /** \brief Set seed of random function.
        * \param seed
        */
      inline void
      setSeed (unsigned int seed)
      {
        seed_ = seed;
      }

      /** \brief Get the value of the internal \a seed parameter.
        */
      inline unsigned int
      getSeed ()
      {
        return (seed_);
      }

    protected:

      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output)
      {
          unsigned N = static_cast<unsigned> (input_->size ());

          // If sample size is 0 or if the sample size is greater then input cloud size
          //   then return entire copy of cloud
          if (sample_ >= N)
          {
            output = *input_;
          }
          else
          {
            // Resize output cloud to sample size
            output.points.resize (sample_);
            output.width = sample_;
            output.height = 1;

            // Set random seed so derived indices are the same each time the filter runs
            std::srand (seed_);
        
            float one_over_N = 0.0;
            int top = N - sample_; //N are the remaining number of elements, n the remaining number of wanted samples
            unsigned index = 0;
            unsigned i = 0;

            // Algorithm A
            for (size_t n = sample_; n > 1; n--)
            {
                one_over_N = 1.0f / float(N); //we need to re-calculate N^{-1}

                float V = std::max(0.001f, unifRand ()); //the 0.001 lower bound has been introduced to avoid random huge areas without selected point.
                unsigned S = 0;
                float quot = float(top) / float(N);

                while( quot > V )
                {
                    S++;
                    N--;
                    top--;
                    quot = quot * ( float(top) / float(N) );
                }

                N--; //this together with N-- above is the same than N - S - 1 (paper Vit84)
                index += S;
                output.points[i++] = input_->points[index++];
            }

            if(N > 0)
            {
                index += static_cast<unsigned int>( unifRand() * N); // * static_cast<unsigned> (unifRand ());
                output.points[i++] = input_->points[index++];
            }
            else
            {
                output.points.resize ( sample_-1 );
            }


            /*for (size_t n = sample_; n >= 2; n--)
            {
              float V = unifRand ();
              unsigned S = 0;
              float quot = float (top) * one_over_N;
              while (quot > V)
              {
                S++;
                top--;
                N--;
                quot = quot * float (top) * one_over_N;
              }
              index += S;
              output.points[i++] = input_->points[index++];
              N--;
            }

            index += N * static_cast<unsigned> (unifRand ());
            output.points[i++] = input_->points[index++];*/
          }
      }

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices)
      {

      size_t N = static_cast<size_t> (input_->size());
      float one_over_N = 1.0f / float (N);

      // If sample size is 0 or if the sample size is greater then input cloud size
      //   then return all indices
      if (sample_ >= N)
      {
        indices = *indices_;
      }
      else
      {
        // Resize output indices to sample size
        indices.resize (sample_);

        // Set random seed so derived indices are the same each time the filter runs
        std::srand (seed_);

        // Algorithm A
        size_t top = 0; //N - sample_;
        unsigned i = 0;
        unsigned index = 0;

        for (size_t n = sample_; n >= 2; n--)
        {
            top = N - n;
            one_over_N = 1.0f / float(N);

            float V = unifRand ();
            unsigned S = 0;
            float quot = float(top) * one_over_N;

            while( quot > V )
            {
                S++;
                N--;
                quot = quot * ( float(top) * one_over_N );
            }

            N--; //this together with N-- above is the same than N - S - 1 (paper Vit84)
            index += S;
            indices[i++] = (*indices_)[index++];

        }

        index += static_cast<unsigned int>( unifRand() * N);
        indices[i++] = (*indices_)[index++];
      }

      }

      /** \brief Return a random number fast using a LCG (Linear Congruential Generator) algorithm.
        * See http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/ for more information.
        */
      inline float
      unifRand ()
      {
        return (static_cast<float>(rand () / double (RAND_MAX)));
        //return (((214013 * seed_ + 2531011) >> 16) & 0x7FFF);
      }
  };

  

#endif  //#ifndef PCL_FILTERS_RANDOM_SUBSAMPLE_CORRECTED_H_
