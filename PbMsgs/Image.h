#ifndef IMAGE_H
#define IMAGE_H

#include <fstream>

#include <Eigen/Eigen>
#include <PbMsgs/Messages.pb.h>

#define HAVE_OPENCV
#ifdef HAVE_OPENCV
#include <opencv.hpp>
#endif



namespace pb {


#ifdef HAVE_OPENCV

void ReadCvMat( const cv::Mat& cvImage, pb::ImageMsg* pbImage )
{
    pbImage->set_data( (const char*)cvImage.data );
    pbImage->set_height( cvImage.rows );
    pbImage->set_width( cvImage.cols );

    if( cvImage.elemSize1() == 1 ) {
        pbImage->set_type( pb::ImageMsg_Type_PB_UNSIGNED_BYTE );
    }
    if( cvImage.elemSize1() == 2 ) {
        pbImage->set_type( pb::ImageMsg_Type_PB_UNSIGNED_SHORT );
    }
    if( cvImage.elemSize1() == 4 ) {
        pbImage->set_type( pb::ImageMsg_Type_PB_FLOAT );
    }

    if( cvImage.channels() == 1 ) {
        pbImage->set_format( pb::ImageMsg_Format_PB_LUMINANCE );
    }
    if( cvImage.channels() == 3 ) {
        pbImage->set_format( pb::ImageMsg_Format_PB_RGB );
    }
}

cv::Mat WriteCvMat( pb::ImageMsg* pbImage )
{
    int nCvType;
    if( pbImage->type() == pb::ImageMsg_Type_PB_BYTE || pbImage->type() == pb::ImageMsg_Type_PB_UNSIGNED_BYTE ) {
        if(pbImage->format() == pb::ImageMsg_Format_PB_LUMINANCE ) {
            nCvType = CV_8UC1;
        }
        if(pbImage->format() == pb::ImageMsg_Format_PB_RGB ) {
            nCvType = CV_8UC3;
        }
    }
    if( pbImage->type() == pb::ImageMsg_Type_PB_UNSIGNED_SHORT || pbImage->type() == pb::ImageMsg_Type_PB_SHORT ) {
        if(pbImage->format() == pb::ImageMsg_Format_PB_LUMINANCE ) {
            nCvType = CV_16UC1;
        }
        if(pbImage->format() == pb::ImageMsg_Format_PB_RGB ) {
            nCvType = CV_16UC3;
        }
    }
    if( pbImage->type() == pb::ImageMsg_Type_PB_FLOAT ) {
        if(pbImage->format() == pb::ImageMsg_Format_PB_LUMINANCE ) {
            nCvType = CV_32FC1;
        }
        if(pbImage->format() == pb::ImageMsg_Format_PB_RGB ) {
            nCvType = CV_32FC3;
        }
    }

    return cv::Mat( pbImage->height(), pbImage->width(), nCvType, (void*)pbImage->mutable_data()->data() );
}

void ReadFile( const std::string sFileName, pb::ImageMsg* pbImage )
{
    cv::Mat Image;

    std::string sExtension = sFileName.substr( sFileName.rfind( "." ) + 1 );

    // check if it is our own "portable depth map" format
    if( sExtension == "pdm" ) {

        // magic number P7, portable depthmap, binary
        std::ifstream File( sFileName.c_str() );

        unsigned int        nImgWidth;
        unsigned int        nImgHeight;
        long unsigned int   nImgSize;

        if( File.is_open() ) {
            std::string sType;
            File >> sType;
            File >> nImgWidth;
            File >> nImgHeight;
            File >> nImgSize;

            // the actual PGM/PPM expects this as the next field:
            //		nImgSize++;
            //		nImgSize = (log( nImgSize ) / log(2)) / 8.0;

            // but ours has the actual size (4 bytes of float * pixels):
            nImgSize = 4 * nImgWidth * nImgHeight;

            Image.create( nImgHeight, nImgWidth, CV_32FC1 );

            File.seekg( File.tellg() + (std::ifstream::pos_type)1, std::ios::beg );
            File.read( (char*)Image.data, nImgSize );
            File.close();
        }
    } else {
        // ... otherwise let OpenCV open it
        Image = cv::imread( sFileName, cv::IMREAD_UNCHANGED );
    }
    ReadCvMat( Image, pbImage );
}

#endif



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Image
{
public:
    Image() {}

    unsigned int width()
    {
        return m_pImage->width();
    }

    unsigned int height()
    {
        return m_pImage->height();
    }

    const char* data()
    {
        return m_pImage->mutable_data()->data();
    }

#ifdef HAVE_OPENCV
    cv::Mat cvMat()
    {
        return WriteCvMat(m_pImage);
    }
#endif

    friend class ImageArray;

private:
    void _UpdatePointer( ImageMsg* Ptr )
    {
        m_pImage = Ptr;
    }


private:
    ImageMsg*       m_pImage;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ImageArray
{
public:
    ImageArray() {}

    CameraMsg& ref()
    {
        return m_Message;
    }

    unsigned int numImages()
    {
        return m_Message.image_size();
    }

    Image& operator()( unsigned int idx = 0 )
    {
        if( idx < numImages() ) {
            return *(m_vImages[idx]);
        }
        std::cerr << "error: Image index out of bounds." << std::endl;
        exit(1);
    }

    void SelfUpdate()
    {
        if( m_vImages.size() == 0 ) {
            for( unsigned int ii = 0; ii < numImages(); ++ii ) {
                m_vImages.push_back( new Image() );
            }
        }

        for( unsigned int ii = 0; ii < numImages(); ++ii ) {
            m_vImages[ii]->_UpdatePointer( m_Message.mutable_image(ii) );
        }
    }


private:
    CameraMsg                       m_Message;
    std::vector< Image* >           m_vImages;
};



}


#endif // IMAGE_H