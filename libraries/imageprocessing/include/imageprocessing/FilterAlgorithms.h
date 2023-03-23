#ifndef CURAN_FILTER_ALGORITHMS_HEADER_FILE_
#define CURAN_FILTER_ALGORITHMS_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "itkBinaryThresholdImageFilter.h"
#include <vector>
#include "utils/Lockable.h"
#include <memory>

namespace curan {
    namespace image {
        namespace filtering {

            class Implementation {
            public:
                virtual InternalImageType::Pointer update() = 0;
                virtual InternalImageType* get_output() = 0;
                virtual void set_input(const InternalImageType*) = 0;
            };

            class Filter : utils::Lockable<Filter> {

                InternalImageType::Pointer input;
                InternalImageType::Pointer output;

                std::list<std::shared_ptr<Implementation>> filters;

            public:

                void operator<< (std::shared_ptr<Implementation>);
            
                InternalImageType::Pointer get_input();
                InternalImageType::Pointer get_output();
            };
            
            class ThreholdFilter : public Implementation, utils::Lockable<ThreholdFilter> {
            public:

                struct Info {
                    char_pixel_type lower_bound = 0;
                    char_pixel_type upper_bound = 255;
                    char_pixel_type outside_value = 0;
                };

            private:

                Filter* owner = nullptr;

                using FilterType = itk::ThresholdImageFilter<InternalImageType>;
                FilterType::Pointer filter;
                char_pixel_type lower_bound = 0;
                char_pixel_type upper_bound = 255;

                ThreholdFilter(Info& info);

                friend Filter;

            public:

                std::shared_ptr<ThreholdFilter> make(Info& info);

                void updateinfo(Info& info);

                InternalImageType::Pointer update() override;
                InternalImageType* get_output() override;
                void set_input(const InternalImageType*) override;
            };

            class CannyFilter : public Implementation, utils::Lockable<CannyFilter> {
            public:

                struct Info {
                    
                };

            private:

                Filter* owner = nullptr;

                using real_pixel_type = double;
                using CastToRealFilterType = itk::CastImageFilter<char_pixel_type, real_pixel_type>;
                using CannyFilterType = itk::CannyEdgeDetectionImageFilter<real_pixel_type, real_pixel_type>;
                using RescaleFilterType = itk::RescaleIntensityImageFilter<real_pixel_type, char_pixel_type>;

                CannyFilter(Info& info);

                friend Filter;

            public:

                std::shared_ptr<CannyFilter> make(Info& info);

                void updateinfo(Info& info);
               
                InternalImageType::Pointer update() override;
                InternalImageType* get_output() override;
                void set_input(const InternalImageType*) override;
            };

            class BinarizeFilter : public Implementation, utils::Lockable<BinarizeFilter> {
            public:

                struct Info {
                    char_pixel_type lower_bound = 0;
                    char_pixel_type upper_bound = 255;
                    Filter* owner = nullptr;
                };

            private:

                Filter* owner = nullptr;

                char_pixel_type lower_bound = 0;
                char_pixel_type upper_bound = 255;
                using FilterType = itk::BinaryThresholdImageFilter<InternalImageType, InternalImageType>;
                FilterType::Pointer filter;

                BinarizeFilter(Info& info);

                friend Filter;

            public:

                std::shared_ptr<BinarizeFilter> make(Info& info);

                void updateinfo(Info& info);

                InternalImageType::Pointer update() override;
                InternalImageType* get_output() override;
                void set_input(const InternalImageType*) override;
            };


        }
    }
}

#endif