//
// Created by quan on 2021/12/22.
//

#ifndef XSLAM_TUTORIAL_BRIEF_H
#define XSLAM_TUTORIAL_BRIEF_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/dynamic_bitset.hpp>

namespace xslam {
namespace opencv {

class BRIEF
{
public:
    // Bitset type
    typedef boost::dynamic_bitset<> bitset;

    // Type of pairs
    enum Type
    {
        RANDOM, // random pairs (Calonder's original version)
        RANDOM_CLOSE, // random but close pairs (used in GalvezIROS11)
    };

    BRIEF(int nbits = 256, int patch_size = 48, Type type = RANDOM_CLOSE);
    virtual ~BRIEF();


    inline int getDescriptorLengthInBits() const
    {
        return m_bit_length;
    }

    inline Type getType() const
    {
        return m_type;
    }
  
    inline int getPatchSize() const
    {
        return m_patch_size;
    }
    
    inline void operator() (const cv::Mat &image, 
        const std::vector<cv::KeyPoint> &points,
        std::vector<bitset> &descriptors,
        bool treat_image = true) const
    {
        compute(image, points, descriptors, treat_image);
    }
  
    void compute(const cv::Mat &image,
        const std::vector<cv::KeyPoint> &points,
        std::vector<bitset> &descriptors,
        bool treat_image = true) const;

    inline void exportPairs(std::vector<int> &x1, std::vector<int> &y1,
        std::vector<int> &x2, std::vector<int> &y2) const
    {
        x1 = m_x1;
        y1 = m_y1;
        x2 = m_x2;
        y2 = m_y2;
    }
    

    inline void importPairs(const std::vector<int> &x1, 
        const std::vector<int> &y1, const std::vector<int> &x2, 
        const std::vector<int> &y2)
    {
        m_x1 = x1;
        m_y1 = y1;
        m_x2 = x2;
        m_y2 = y2;
        m_bit_length = x1.size();
    }
    

    inline static int distance(const bitset &a, const bitset &b)
    {
        return (a^b).count();
    }

    void generateTestPoints();

protected:
    // Descriptor length in bits
    int m_bit_length;

    // Patch size
    int m_patch_size;

    // Type of pairs
    Type m_type;

    // Coordinates of test points relative to the center of the patch
    std::vector<int> m_x1, m_x2;
    std::vector<int> m_y1, m_y2;

};

} // namespace opencv
} // namespace xslam

#endif //XSLAM_TUTORIAL_BRIEF_H
