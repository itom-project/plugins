//
//
//#include "random_sample_corrected.h"
//
////#ifndef PCL_FILTERS_IMPL_RANDOM_SAMPLE_CORRECTED_H_
////#define PCL_FILTERS_IMPL_RANDOM_SAMPLE_CORRECTED_H_
//
//
/////////////////////////////////////////////////////////////////////////////////
//template<typename PointT> void
//RandomSampleCorrected<PointT>::applyFilter (PointCloud &output)
//{
//  unsigned N = static_cast<unsigned> (input_->size ());
//  float one_over_N = 1.0f / float (N);
//
//  // If sample size is 0 or if the sample size is greater then input cloud size
//  //   then return entire copy of cloud
//  if (sample_ >= N)
//  {
//    output = *input_;
//  }
//  else
//  {
//    // Resize output cloud to sample size
//    output.points.resize (sample_);
//    output.width = sample_;
//    output.height = 1;
//
//    // Set random seed so derived indices are the same each time the filter runs
//    std::srand (seed_);
//
//    unsigned top = N - sample_; // (N-n) from paper Vit84
//    unsigned i = 0;
//    unsigned index = 0;
//
//    // Algorithm A
//    for (size_t n = sample_; n >= 2; n--)
//    {
//      float V = unifRand ();
//      unsigned S = 0;
//      float quot = float (top) * one_over_N;
//      while (quot > V)
//      {
//        S++;
//        top--;
//        N--;
//        quot = quot * float (top) * one_over_N;
//      }
//      index += S;
//      output.points[i++] = input_->points[index++];
//      N--;
//    }
//
//    index += N * static_cast<unsigned> (unifRand ());
//    output.points[i++] = input_->points[index++];
//  }
//}
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//template<typename PointT>
//void
//RandomSampleCorrected<PointT>::applyFilter (std::vector<int> &indices)
//{
//  unsigned N = static_cast<unsigned> (input_->size ());
//  float one_over_N = 1.0f / float (N);
//
//  // If sample size is 0 or if the sample size is greater then input cloud size
//  //   then return all indices
//  if (sample_ >= N)
//  {
//    indices = *indices_;
//  }
//  else
//  {
//    // Resize output indices to sample size
//    indices.resize (sample_);
//
//    // Set random seed so derived indices are the same each time the filter runs
//    std::srand (seed_);
//
//    // Algorithm A
//    unsigned top = N - sample_;
//    unsigned i = 0;
//    unsigned index = 0;
//
//    for (size_t n = sample_; n >= 2; n--)
//    {
//      float V = unifRand ();
//      unsigned S = 0;
//      float quot = float (top) * one_over_N;
//      while (quot > V)
//      {
//        S++;
//        top--;
//        N--;
//        quot = quot * float (top) * one_over_N;
//      }
//      index += S;
//      indices[i++] = (*indices_)[index++];
//      N--;
//    }
//
//    index += N * static_cast<unsigned> (unifRand ());
//    indices[i++] = (*indices_)[index++];
//  }
//}
//
//#define PCL_INSTANTIATE_RandomSampleCorrected(T) template class RandomSampleCorrected<T>;
//
////#endif    // PCL_FILTERS_IMPL_RANDOM_SAMPLE_CORRECTED_H_