#ifndef CURAN_SEGMENTATION_HEADER_FILE_
#define CURAN_SEGMENTATION_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "itkBinaryThresholdImageFilter.h"
#include <vector>
#include "utils/Lockable.h"

namespace curan {
    namespace image {
        namespace filtering {

            class Filter {


            };

            class CompositFilter {
                std::vector<std::shared_ptr<Filter>> filters;

            };
            
            class ThreholdFilter : public Filter , utils::Lockable<ThreholdFilter> {
            public:

                struct Info {
                    char_pixel_type lower_bound = 0;
                    char_pixel_type upper_bound = 255;
                };

            private:

                using FilterType = itk::BinaryThresholdImageFilter<InternalImageType, InternalImageType>;
                Image output;
                FilterType::Pointer filter;

                ThreholdFilter(Info& info);

            public:

                std::shared_ptr<ThreholdFilter> make(Info& info);

                void submit_image(Image input);

                void update(Info& info);
            };

            class SobelFilter : public Filter, utils::Lockable<SobelFilter> {
            public:

                struct Info {

                };

            private:
                
                Image output;

                SobelFilter(Info& info);

            public:

                std::shared_ptr<SobelFilter> make(Info& info);

                void submit_image(Image input);

                void update(Info& info);
               

            };


        }
    }
}

#endif