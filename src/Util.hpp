#pragma once

#include <iostream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>

namespace util
{
    class RandomGenerator
    {
    public:
        RandomGenerator()
            : rng()
        { }
        RandomGenerator(uint64 seed)
            : rng(seed)
        { }

        /** Fills the given vector with independent, uniformly distributed samples.
        */
        template <class VecType>
        void drawUniformVector(
            VecType &v,
            const double unif_min = 0,
            const double unif_max = 1 )
        {
            const size_t N = v.size();
            for (size_t c = 0; c < N; c++)
                v[c] = static_cast<typename VecType::value_type>( rng.uniform(unif_min, unif_max) );
        }

        /** Returns a subset of [0 ... n] set
        */
        void drawUniformSubset(
            const unsigned n,
            std::vector<unsigned> &out_indices
            )
        {
            out_indices.clear();

            for(size_t i = 0; i < n; ++i)
                if( rng() % 2 )
                    out_indices.push_back(i);
        }

        /** Returns random k elements of [0 ... n] set
        */
        void drawUniformSubset(
            const unsigned n,
            const unsigned k,
            std::vector<unsigned> &out_indices
            )
        {
            std::vector<unsigned> set(n);

            for(size_t i = 0; i < n; ++i)
                set[i] = i;

            for(size_t i = 0; i < k; ++i)
                std::swap( set[i], set[i + rng() % (n-i)] );

            set.resize(k);
            out_indices = set;
        }
    private:
        cv::RNG rng;
    };

    void getSubVectors(
        const std::vector<cv::Point3f> modelPoints3D,
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<unsigned> subset,
        std::vector<cv::Point3f> &out_subModelPoints3D,
        std::vector<cv::Point2f> &out_subFoundBoxPoints2D);

    void printVector(std::vector<unsigned>& vector);
}