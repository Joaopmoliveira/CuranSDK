#include "rendering/integrated_shaders/VolumeIntegratedShaders.h"
#include "imageprocessing/TemplatedVolumeAlgorithms.h"
#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "itkImportImageFilter.h"

namespace curan{
namespace image {

IntegratedReconstructor::Info::Info(std::array<double,3> inspacing,std::array<double,3> inorigin, std::array<double,3> insize, std::array<std::array<double,3>,3> indirection){
	auto inextent = gte::Vector3<double>{insize[0]/2.0,insize[1]/2.0,insize[2]/2.0};

    auto origin = gte::Vector3<double>{inorigin[0],inorigin[1],inorigin[2]};
	std::array<gte::Vector3<double>, 3> alignement;
	alignement[0] = {indirection[0][0],indirection[0][1],indirection[0][2]};
	alignement[1] = {indirection[1][0],indirection[1][1],indirection[1][2]};
	alignement[2] = {indirection[2][0],indirection[2][1],indirection[2][2]};

    auto output_origin = origin
		+ alignement[0] * inextent[0]
		+ alignement[1] * inextent[1]
		+ alignement[2] * inextent[2];

    volumetric_bounding_box = gte::OrientedBox3<double>{output_origin,alignement,inextent};
    spacing[0] = inspacing[0];
    spacing[1] = inspacing[1];
    spacing[2] = inspacing[2];
}

vsg::ref_ptr<renderable::Renderable> IntegratedReconstructor::make(Info& info){
    vsg::ref_ptr<IntegratedReconstructor> sphere_to_add = IntegratedReconstructor::create(info);
    vsg::ref_ptr<renderable::Renderable> val = sphere_to_add.cast<renderable::Renderable>();
    return val;
}



IntegratedReconstructor::IntegratedReconstructor(const Info& info) : output_spacing{info.spacing},volumetric_bounding_box{info.volumetric_bounding_box}{
	output_type::IndexType output_start;
	output_start[0] = 0;
    output_start[1] = 0;
    output_start[2] = 0;

    output_type::DirectionType output_directorion;
    output_directorion[0][0] = volumetric_bounding_box.axis[0][0];
    output_directorion[1][0] = volumetric_bounding_box.axis[1][0];
    output_directorion[2][0] = volumetric_bounding_box.axis[2][0];

    output_directorion[0][1] = volumetric_bounding_box.axis[0][1];
    output_directorion[1][1] = volumetric_bounding_box.axis[1][1];
    output_directorion[2][1] = volumetric_bounding_box.axis[2][1];

    output_directorion[0][2] = volumetric_bounding_box.axis[0][2];
    output_directorion[1][2] = volumetric_bounding_box.axis[1][2];
    output_directorion[2][2] = volumetric_bounding_box.axis[2][2];

    output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
    output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
    output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

	vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = volumetric_bounding_box.center[0];
    position_of_center_in_global_frame[1] = volumetric_bounding_box.center[1];
    position_of_center_in_global_frame[2] = volumetric_bounding_box.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = volumetric_bounding_box.extent[0];
    position_in_local_box_frame[1] = volumetric_bounding_box.extent[1];
    position_in_local_box_frame[2] = volumetric_bounding_box.extent[2]; 

    vsg::dmat3 rotation_0_1;

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            rotation_0_1(col,row) = volumetric_bounding_box.axis[col][row];
        

    auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;

    output_type::PointType output_origin;
    output_origin[0] = global_corner_position[0];
    output_origin[1] = global_corner_position[1];	
    output_origin[2] = global_corner_position[2];
    

    vsg::vec3 origin = vsg::vec3(0.0,0.0,0.0);
    vsg::vec3 dx = vsg::vec3(1.0, 0.0, 0.0);
    vsg::vec3 dy = vsg::vec3(0.0, 1.0, 0.0);
    vsg::vec3 dz = vsg::vec3(0.0, 0.0, 1.0);
    vsg::vec3 v000(origin);
    vsg::vec3 v100(origin + dx);
    vsg::vec3 v110(origin + dx + dy);
    vsg::vec3 v010(origin + dy);
    vsg::vec3 v001(origin + dz);
    vsg::vec3 v101(origin + dx + dz);
    vsg::vec3 v111(origin + dx + dy + dz);
    vsg::vec3 v011(origin + dy + dz);

    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::ushortArray> indices;

    // set up vertex and index arrays
    vertices = vsg::vec3Array::create(
        { v000, v100, v101, v001,   // front
         v100, v110, v111, v101,   // right
         v110, v010, v011, v111,   // far
         v010, v000, v001, v011,   // left
         v010, v110, v100, v000,   // bottom
         v001, v101, v111, v011 }); // top

    indices = vsg::ushortArray::create(
        { 0, 1, 2, 0, 2, 3,
         4, 5, 6, 4, 6, 7,
         8, 9, 10, 8, 10, 11,
         12, 13, 14, 12, 14, 15,
         16, 17, 18, 16, 18, 19,
         20, 21, 22, 20, 22, 23 });

    vsg::DataList arrays;
    arrays.push_back(vertices);

    auto vid = vsg::VertexIndexDraw::create();
    vid->assignArrays(arrays);
    vid->assignIndices(indices);
    vid->indexCount = static_cast<uint32_t>(indices->size());
    uint32_t instanceCount = 1;
    vid->instanceCount = instanceCount;

    float TransparencyValue = 0.5f;
    float AlphaFuncValue = 0.1f;
    float SampleDensityValue = 0.005f;

    float size_x = 1.0;
    float size_y = 1.0;
    float size_z = 1.0;

    vsg::ShaderStage::SpecializationConstants specializationVertexContexts{
        {0, vsg::floatValue::create(size_x)},
        {1, vsg::floatValue::create(size_y)},
        {2, vsg::floatValue::create(size_z)}
    };

    vsg::ShaderStage::SpecializationConstants specializationContexts{
    {0, vsg::floatValue::create(TransparencyValue)},
    {1, vsg::floatValue::create(AlphaFuncValue)},
    {2, vsg::floatValue::create(SampleDensityValue)}
    };

       // load shaders
    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", renderable::volume_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", renderable::volume_frag);

    if (!vertexShader || !fragmentShader)
        throw std::runtime_error("failed to create the shaders necessary for the volume rendering");
    
    fragmentShader->specializationConstants = specializationContexts;
    vertexShader->specializationConstants = specializationVertexContexts;

    // read texture image
    textureData = vsg::floatArray3D::create(output_size[0], output_size[1], output_size[2]);
    textureData->properties.format = VK_FORMAT_R32_SFLOAT;
    textureData->properties.dataVariance = vsg::DYNAMIC_DATA;

   // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection view, and model matrices, actual push constant calls automatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, // vertex data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, // vertex data
    };

    auto rasterizationState = vsg::RasterizationState::create();
    rasterizationState->cullMode = VK_CULL_MODE_FRONT_BIT;

    vsg::GraphicsPipelineStates pipelineStates{
      vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
      vsg::InputAssemblyState::create(),
      rasterizationState,
      vsg::MultisampleState::create(),
      vsg::ColorBlendState::create(),
      vsg::DepthStencilState::create() };

    auto pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{ descriptorSetLayout }, pushConstantRanges);
    auto graphicsPipeline = vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{ vertexShader, fragmentShader }, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto clampToEdge_sampler = vsg::Sampler::create();
    clampToEdge_sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    clampToEdge_sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    clampToEdge_sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    clampToEdge_sampler->borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
    clampToEdge_sampler->anisotropyEnable = VK_TRUE;
    clampToEdge_sampler->maxAnisotropy = 1.0;

    auto texture = vsg::DescriptorImage::create(clampToEdge_sampler, textureData, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{ texture });
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout, 0, descriptorSet);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSet);

    vsg::dvec3 position(0.0f, 0.0f, 0.0f);

    vsg::dvec3 mixture(output_size[0]* info.spacing[0],output_size[1]* info.spacing[1], output_size[2]* info.spacing[2]);

    auto scalling_transform = vsg::MatrixTransform::create(vsg::scale(mixture));

	vsg::dmat4 homogeneous_transformation;
	for(size_t row = 0; row < 3 ; ++row)
		for(size_t col = 0; col < 3; ++col)
			homogeneous_transformation(col,row) = rotation_0_1(col,row);
	homogeneous_transformation(3,0) = output_origin[0];
	homogeneous_transformation(3,1) = output_origin[1];
	homogeneous_transformation(3,2) = output_origin[2];
	homogeneous_transformation(3,3) = 1.0;
    transform = vsg::MatrixTransform::create(homogeneous_transformation);
    
    // add geometry
    scenegraph->addChild(vid);
    scalling_transform->addChild(scenegraph);

    obj_contained = vsg::Group::create();
    obj_contained->addChild(scalling_transform);

    output_type::RegionType output_region;
    output_region.SetSize(output_size);
    output_region.SetIndex(output_start);

	acummulation_buffer = accumulator_type::New();
	acummulation_buffer->SetRegions(output_region);
	acummulation_buffer->SetOrigin(output_origin);
	acummulation_buffer->SetSpacing(output_spacing);
	acummulation_buffer->SetDirection(output_directorion);
	acummulation_buffer->Allocate(true);

    if (info.identifier)
        set_identifier(*info.identifier);
}

IntegratedReconstructor::~IntegratedReconstructor(){

}

IntegratedReconstructor& IntegratedReconstructor::set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy){
	std::lock_guard<std::mutex> g{mut};
    interpolation_strategy = new_interpolation_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_fillstrategy(const curan::image::reconstruction::FillingStrategy& new_filling_strategy){
	std::lock_guard<std::mutex> g{mut};
    fillType = new_filling_strategy;
    return *(this);
}

void IntegratedReconstructor::add_kernel_descritor(curan::image::reconstruction::KernelDescriptor descriptor){
	
	if (fillType == descriptor.fillType) {
		descriptor.Allocate();
		kernels.push_back(descriptor);
	}
}

void IntegratedReconstructor::fill_holes()
{			using PixelType = float;
            using ImportFilterType = itk::ImportImageFilter<PixelType, 3>;
            auto importFilter = ImportFilterType::New();
            ImportFilterType::IndexType start;
            start.Fill(0);

			vsg::dvec3 position_of_center_in_global_frame;
			position_of_center_in_global_frame[0] = volumetric_bounding_box.center[0];
			position_of_center_in_global_frame[1] = volumetric_bounding_box.center[1];
			position_of_center_in_global_frame[2] = volumetric_bounding_box.center[2];

			vsg::dvec3 position_in_local_box_frame;
			position_in_local_box_frame[0] = volumetric_bounding_box.extent[0];
			position_in_local_box_frame[1] = volumetric_bounding_box.extent[1];
			position_in_local_box_frame[2] = volumetric_bounding_box.extent[2]; 

			vsg::dmat3 rotation_0_1;

			for(size_t col = 0; col < 3; ++col)
				for(size_t row = 0; row < 3; ++row)
					rotation_0_1(col,row) = volumetric_bounding_box.axis[col][row];
				

			auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;

            const itk::SpacePrecisionType output_origin[3] ={global_corner_position[0],global_corner_position[1],global_corner_position[2]};
            itk::Matrix<double>  ref_to_output_origin;
           	ref_to_output_origin(0, 0) = volumetric_bounding_box.axis[0][0];
			ref_to_output_origin(1, 0) = volumetric_bounding_box.axis[0][1];
			ref_to_output_origin(2, 0) = volumetric_bounding_box.axis[0][2];
		
			ref_to_output_origin(0, 1) = volumetric_bounding_box.axis[1][0];
			ref_to_output_origin(1, 1) = volumetric_bounding_box.axis[1][1];
			ref_to_output_origin(2, 1) = volumetric_bounding_box.axis[1][2];
		
			ref_to_output_origin(0, 2) = volumetric_bounding_box.axis[2][0];
			ref_to_output_origin(1, 2) = volumetric_bounding_box.axis[2][1];
			ref_to_output_origin(2, 2) = volumetric_bounding_box.axis[2][2];
		

            ImportFilterType::RegionType region;
            region.SetIndex(start);
            region.SetSize(output_size);
            importFilter->SetRegion(region);
           	importFilter->SetSpacing(output_origin);
            importFilter->SetSpacing(output_spacing);
			importFilter->SetDirection(ref_to_output_origin);
            const unsigned int numberOfPixels = output_size[0] * output_size[1] * output_size[2];
	
			double max_val1 = -100000;

            const bool importImageFilterWillOwnTheBuffer = false;
            float * my_beatiful_pointer = textureData->data();

			for(auto iter_val = textureData->begin();iter_val!=textureData->end(); ++iter_val)
				max_val1 = (*iter_val>max_val1) ? *iter_val : max_val1;

            importFilter->SetImportPointer(my_beatiful_pointer, numberOfPixels, importImageFilterWillOwnTheBuffer);
            using RescaleFilterType = itk::RescaleIntensityImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>;
            using CastFilterType = itk::CastImageFilter<itk::Image<float, 3>, itk::Image<unsigned char, 3>>;

            RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
            CastFilterType::Pointer castFilter = CastFilterType::New();

            rescaleFilter->SetInput(importFilter->GetOutput());
            rescaleFilter->SetOutputMaximum(255.0);
            rescaleFilter->SetOutputMinimum(0.0);
            castFilter->SetInput(rescaleFilter->GetOutput());
            castFilter->Update();

            auto volume_temp = castFilter->GetOutput();

			char_pixel_type* inVolPtr = volume_temp->GetBufferPointer();
			short_pixel_type* accPtr = acummulation_buffer->GetBufferPointer();

			//we need to create the output volume where the 
			//voxels will be placed after the filling procedure is over
			itk::ImageDuplicator<InternalImageType>::Pointer duplicator = itk::ImageDuplicator<InternalImageType>::New();
			duplicator->SetInputImage(volume_temp);
			duplicator->Update();
			InternalImageType::Pointer filled_volume = duplicator->GetOutput();
			char_pixel_type* outPtr = filled_volume->GetBufferPointer();
			auto outdata_ROI = volume_temp->GetLargestPossibleRegion();
			auto size_out = outdata_ROI.GetSize();
			auto start_out = outdata_ROI.GetIndex();

			uint64_t outExt[6];
			outExt[0] = start_out[0];
			outExt[1] = size_out[0] - 1;

			outExt[2] = start_out[1];
			outExt[3] = size_out[1] - 1;

			outExt[4] = start_out[2];
			outExt[5] = size_out[2] - 1;

			// get increments for volume and for accumulation buffer
			uint64_t byteIncVol[3] = { 0 }; //x,y,z

			int idx;
			uint64_t incr = reconstruction::INPUT_COMPONENTS;
			for (idx = 0; idx < 3; ++idx)
			{
				byteIncVol[idx] = incr;
				incr *= (outExt[idx * 2 + 1] - outExt[idx * 2] + 1);
			}

			// this will store the position of the pixel being looked at currently
			uint64_t currentPos[3]; //x,y,z

			uint64_t numVolumeComponents = reconstruction::INPUT_COMPONENTS;

			// Set interpolation method - nearest neighbor or trilinear
			bool (*apply)(char_pixel_type * inputData,
				unsigned short* accData,
				uint64_t * inputOffsets,
				uint64_t * bounds,
				uint64_t * wholeExtent,
				uint64_t * thisPixel,
				char_pixel_type & returnVal,
				const reconstruction::KernelDescriptor * descriptor) = NULL;

			switch (fillType)
			{
			case reconstruction::FillingStrategy::GAUSSIAN:
				apply = &reconstruction::ApplyGaussian;
				break;
			case reconstruction::FillingStrategy::GAUSSIAN_ACCUMULATION:
				apply = &reconstruction::ApplyGaussianAccumulation;
				break;
			case reconstruction::FillingStrategy::DISTANCE_WEIGHT_INVERSE:
				apply = &reconstruction::ApplyDistanceWeightInverse;
				break;
			case reconstruction::FillingStrategy::NEAREST_NEIGHBOR:
				apply = &reconstruction::ApplyNearestNeighbor;
				break;
			case reconstruction::FillingStrategy::STICK:
				apply = &reconstruction::ApplySticks;
				break;
			default:
			{
				std::string s = "Unknown interpolation mode: " + std::to_string(fillType);
				//utilities::cout << s;
				return;
			}
			}
			// iterate through each voxel. When the accumulation buffer is 0, fill that hole, and continue.
			for (currentPos[2] = outExt[4]; currentPos[2] <= outExt[5]; currentPos[2]++)
			{
				for (currentPos[1] = outExt[2]; currentPos[1] <= outExt[3]; currentPos[1]++)
				{
					for (currentPos[0] = outExt[0]; currentPos[0] <= outExt[1]; currentPos[0]++)
					{
						// accumulator index should not depend on which individual component is being interpolated
						int accIndex = (currentPos[0] * byteIncVol[0]) + (currentPos[1] * byteIncVol[1]) + (currentPos[2] * byteIncVol[2]);
						if (accPtr[accIndex] == 0) // if not hit by accumulation during vtkIGSIOPasteSliceIntoVolume
						{
							bool result(false);
							for (const auto kernel : kernels)
							{
								result = apply(inVolPtr, accPtr, byteIncVol, outExt, outExt, currentPos, outPtr[accIndex], &kernel);
								if (result) {
									break;
								} // end checking interpolation success
							}
						} else // if hit, just use the apparent value
						{
							int volCompIndex = (currentPos[0] * byteIncVol[0]) + (currentPos[1] * byteIncVol[1]) + (currentPos[2] * byteIncVol[2]);
							outPtr[volCompIndex] = inVolPtr[volCompIndex];
						} // end accumulation check
					} // end x loop
				} // end y loop
			} // end z loop		

			using CastFilterType1 = itk::CastImageFilter<itk::Image<unsigned char,3> , itk::Image<float, 3>>;
			using RescaleFilterType1 = itk::RescaleIntensityImageFilter<itk::Image<float, 3>, itk::Image<float, 3>>;
			
			CastFilterType1::Pointer new_castFilter = CastFilterType1::New();
            RescaleFilterType1::Pointer new_rescaleFilter = RescaleFilterType1::New();

			new_castFilter->SetInput(filled_volume);

            new_rescaleFilter->SetInput(new_castFilter->GetOutput());
            new_rescaleFilter->SetOutputMaximum(1.0);
            new_rescaleFilter->SetOutputMinimum(0.0);
			new_castFilter->Update();
			auto image=new_castFilter->GetOutput();

			std::printf("\n size of destination: (%d %d %d ) pixel size (%d)\n size of origin : (%d %d %d ) pixel size: (%d)\n copied block: %d",textureData->width(),textureData->height(),textureData->depth(),textureData->stride(),output_size[0],output_size[1],output_size[2],sizeof(float),numberOfPixels*sizeof(float));
			std::printf("\n Image size from itk: %d\n",image->GetLargestPossibleRegion().GetSize()[0]*image->GetLargestPossibleRegion().GetSize()[1]*image->GetLargestPossibleRegion().GetSize()[2]);
			//std::memcpy(textureData->data(), image->GetBufferPointer(), (size_t)(textureData->size()/2.0));
			std::memcpy(textureData->data(), image->GetBufferPointer(), numberOfPixels*sizeof(float));
			textureData->dirty();
		return;
	};

IntegratedReconstructor& IntegratedReconstructor::set_clipping(const Clipping& new_clipping){
	std::lock_guard<std::mutex> g{mut};
    clipping = new_clipping;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::add_frame(input_type::Pointer image_pointer){
	std::lock_guard<std::mutex> g{mut};
	frame_data.push_back(image_pointer);
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::add_frames(std::vector<input_type::Pointer>& images_vector){
	std::lock_guard<std::mutex> g{mut};
    frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
    return *(this);
}

vsg::ref_ptr<vsg::floatArray3D> IntegratedReconstructor::get_texture_data() {
	return textureData;
}

itk::Size<3U> IntegratedReconstructor::get_output_size() {
	return output_size;
}


bool IntegratedReconstructor::update(){
	vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = volumetric_bounding_box.center[0];
    position_of_center_in_global_frame[1] = volumetric_bounding_box.center[1];
    position_of_center_in_global_frame[2] = volumetric_bounding_box.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = volumetric_bounding_box.extent[0];
    position_in_local_box_frame[1] = volumetric_bounding_box.extent[1];
    position_in_local_box_frame[2] = volumetric_bounding_box.extent[2]; 

    vsg::dmat3 rotation_0_1;

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            rotation_0_1(col,row) = volumetric_bounding_box.axis[col][row];
        

    auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;

    output_type::PointType output_origin;
    output_origin[0] = global_corner_position[0];
    output_origin[1] = global_corner_position[1];	
    output_origin[2] = global_corner_position[2];
	
	Eigen::Matrix4d ref_to_output_origin;
	ref_to_output_origin(0, 0) = volumetric_bounding_box.axis[0][0];
	ref_to_output_origin(1, 0) = volumetric_bounding_box.axis[0][1];
	ref_to_output_origin(2, 0) = volumetric_bounding_box.axis[0][2];
	ref_to_output_origin(3, 0) = 0.0;

	ref_to_output_origin(0, 1) = volumetric_bounding_box.axis[1][0];
	ref_to_output_origin(1, 1) = volumetric_bounding_box.axis[1][1];
	ref_to_output_origin(2, 1) = volumetric_bounding_box.axis[1][2];
	ref_to_output_origin(3, 1) = 0.0;

	ref_to_output_origin(0, 2) = volumetric_bounding_box.axis[2][0];
	ref_to_output_origin(1, 2) = volumetric_bounding_box.axis[2][1];
	ref_to_output_origin(2, 2) = volumetric_bounding_box.axis[2][2];
	ref_to_output_origin(3, 2) = 0.0;

	ref_to_output_origin(0, 3) = output_origin[0];
	ref_to_output_origin(1, 3) = output_origin[1];
	ref_to_output_origin(2, 3) = output_origin[2];
	ref_to_output_origin(3, 3) = 1.0;

	Eigen::Matrix4d output_to_ref = ref_to_output_origin.inverse();

	unsigned int accOverflow = 20;

	curan::image::reconstruction::PasteSliceIntoVolumeInsertSliceParamsTemplated<input_pixel_type,output_pixel_type> paste_slice_info;
	paste_slice_info.outPtr = textureData->data();
    auto size_out = acummulation_buffer->GetLargestPossibleRegion().GetSize();
    auto origin_out = acummulation_buffer->GetOrigin();
    auto spacing_out = acummulation_buffer->GetSpacing();
    paste_slice_info.out_origin[0] = origin_out[0];
    paste_slice_info.out_origin[1] = origin_out[1];
    paste_slice_info.out_origin[2] = origin_out[2];
    paste_slice_info.out_size[0] = size_out[0];
    paste_slice_info.out_size[1] = size_out[1];
    paste_slice_info.out_size[2] = size_out[2];
    paste_slice_info.out_spacing[0] = spacing_out[0];
    paste_slice_info.out_spacing[1] = spacing_out[1];
    paste_slice_info.out_spacing[2] = spacing_out[2];
	paste_slice_info.accPtr = acummulation_buffer->GetBufferPointer();
	paste_slice_info.interpolationMode = interpolation_strategy;
	paste_slice_info.compoundingMode = compounding_strategy;
	paste_slice_info.accOverflowCount = &accOverflow;
	paste_slice_info.pixelRejectionThreshold = 0;
	paste_slice_info.image_number = 0;

	Eigen::Matrix4d ref_to_image;
	ref_to_image(3, 0) = 0.0;
	ref_to_image(3, 1) = 0.0;
	ref_to_image(3, 2) = 0.0;
	ref_to_image(3, 3) = 1.0;

	// cicle throught all frames and insert
	// them in the output buffer, one at a time

	std::vector<input_type::Pointer> local_image_copies;
	{
		std::lock_guard<std::mutex> g{mut};
		local_image_copies = std::move(frame_data);
		frame_data = std::vector<input_type::Pointer>();
		if(local_image_copies.size()==0)
			return false;
	}

	for (auto img : local_image_copies) {	
	    int inputFrameExtentForCurrentThread[6] = { 0, 0, 0, 0, 0, 0 };
		double clipRectangleOrigin [2]; // array size 2
		double clipRectangleSize [2]; // array size 2
		
		auto local_size = img->GetLargestPossibleRegion().GetSize();
		auto local_origin = img->GetOrigin();

		if(clipping){
			clipRectangleOrigin[0] = (*clipping).clipRectangleOrigin[0];
			clipRectangleOrigin[1] = (*clipping).clipRectangleOrigin[1];
			clipRectangleSize[0] = (*clipping).clipRectangleSize[0];
			clipRectangleSize[1] = (*clipping).clipRectangleSize[1];
		} else {
			clipRectangleOrigin[0] = 0;
			clipRectangleOrigin[1] = 0;
			clipRectangleSize[0] = local_size.GetSize()[0]-1;
			clipRectangleSize[1] = local_size.GetSize()[1]-1;
		}

		inputFrameExtentForCurrentThread[1] = local_size.GetSize()[0]-1;
		inputFrameExtentForCurrentThread[3] = local_size.GetSize()[1]-1;
		paste_slice_info.clipRectangleOrigin = clipRectangleOrigin;
	    paste_slice_info.clipRectangleSize = clipRectangleSize;
		paste_slice_info.inExt = inputFrameExtentForCurrentThread;
		paste_slice_info.image_number += 1;

		itk::Matrix<double> image_orientation = img->GetDirection();
		itk::Point<double> image_origin = img->GetOrigin();

		ref_to_image(0, 0) = image_orientation[0][0];
		ref_to_image(1, 0) = image_orientation[1][0];
		ref_to_image(2, 0) = image_orientation[2][0];

		ref_to_image(0, 1) = image_orientation[0][1];
		ref_to_image(1, 1) = image_orientation[1][1];
		ref_to_image(2, 1) = image_orientation[2][1];

		ref_to_image(0, 2) = image_orientation[0][2];
		ref_to_image(1, 2) = image_orientation[1][2];
		ref_to_image(2, 2) = image_orientation[2][2];

		ref_to_image(0, 3) = image_origin[0];
		ref_to_image(1, 3) = image_origin[1];
		ref_to_image(2, 3) = image_origin[2];

		// The matrix is the transformation of the 
		// origin of the output volume (1) to the 
		// origin of the input image (2). This is 
		// given by T02=T01*T12, and by premultiplying 
		// by T10=inverse(T01) we obtain T12=inverse(T01)*T02
		Eigen::Matrix4d output_to_origin = output_to_ref * ref_to_image;

        auto size_in = img->GetLargestPossibleRegion().GetSize();
        auto origin_in = img->GetOrigin();
        auto spacing_in = img->GetSpacing();
		paste_slice_info.inPtr = img->GetBufferPointer();
        paste_slice_info.in_origin[0] = origin_in[0];
        paste_slice_info.in_origin[1] = origin_in[1];
        paste_slice_info.in_origin[2] = origin_in[2];
        paste_slice_info.in_size[0] = size_in[0];
        paste_slice_info.in_size[1] = size_in[1];
        paste_slice_info.in_size[2] = size_in[2];
        paste_slice_info.in_spacing[0] = spacing_in[0];
        paste_slice_info.in_spacing[1] = spacing_in[1];
        paste_slice_info.in_spacing[2] = spacing_in[2];
		paste_slice_info.matrix = output_to_origin;
		curan::image::reconstruction::TemplatedUnoptimizedInsertSlice<input_pixel_type,output_pixel_type,255>(&paste_slice_info);
	};
    textureData->dirty();
    return true;
}

bool IntegratedReconstructor::multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool){
	vsg::dvec3 position_of_center_in_global_frame;
	position_of_center_in_global_frame[0] = volumetric_bounding_box.center[0];
    position_of_center_in_global_frame[1] = volumetric_bounding_box.center[1];
    position_of_center_in_global_frame[2] = volumetric_bounding_box.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = volumetric_bounding_box.extent[0];
    position_in_local_box_frame[1] = volumetric_bounding_box.extent[1];
    position_in_local_box_frame[2] = volumetric_bounding_box.extent[2]; 

    vsg::dmat3 rotation_0_1;

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            rotation_0_1(col,row) = volumetric_bounding_box.axis[col][row];
        

    auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;

    output_type::PointType output_origin;
    output_origin[0] = global_corner_position[0];
    output_origin[1] = global_corner_position[1];	
    output_origin[2] = global_corner_position[2];

	Eigen::Matrix4d ref_to_output_origin;
	ref_to_output_origin(0, 0) = volumetric_bounding_box.axis[0][0];
	ref_to_output_origin(1, 0) = volumetric_bounding_box.axis[0][1];
	ref_to_output_origin(2, 0) = volumetric_bounding_box.axis[0][2];
	ref_to_output_origin(3, 0) = 0.0;

	ref_to_output_origin(0, 1) = volumetric_bounding_box.axis[1][0];
	ref_to_output_origin(1, 1) = volumetric_bounding_box.axis[1][1];
	ref_to_output_origin(2, 1) = volumetric_bounding_box.axis[1][2];
	ref_to_output_origin(3, 1) = 0.0;

	ref_to_output_origin(0, 2) = volumetric_bounding_box.axis[2][0];
	ref_to_output_origin(1, 2) = volumetric_bounding_box.axis[2][1];
	ref_to_output_origin(2, 2) = volumetric_bounding_box.axis[2][2];
	ref_to_output_origin(3, 2) = 0.0;

	ref_to_output_origin(0, 3) = output_origin[0];
	ref_to_output_origin(1, 3) = output_origin[1];
	ref_to_output_origin(2, 3) = output_origin[2];
	ref_to_output_origin(3, 3) = 1.0;

	Eigen::Matrix4d output_to_ref = ref_to_output_origin.inverse();

	unsigned int accOverflow = 20;

	curan::image::reconstruction::PasteSliceIntoVolumeInsertSliceParamsTemplated<input_pixel_type,output_pixel_type> paste_slice_info;
	paste_slice_info.outPtr = textureData->data();
    auto size_out = acummulation_buffer->GetLargestPossibleRegion().GetSize();
    auto origin_out = acummulation_buffer->GetOrigin();
    auto spacing_out = acummulation_buffer->GetSpacing();
    paste_slice_info.out_origin[0] = origin_out[0];
    paste_slice_info.out_origin[1] = origin_out[1];
    paste_slice_info.out_origin[2] = origin_out[2];
    paste_slice_info.out_size[0] = size_out[0];
    paste_slice_info.out_size[1] = size_out[1];
    paste_slice_info.out_size[2] = size_out[2];
    paste_slice_info.out_spacing[0] = spacing_out[0];
    paste_slice_info.out_spacing[1] = spacing_out[1];
    paste_slice_info.out_spacing[2] = spacing_out[2];
	paste_slice_info.accPtr = acummulation_buffer->GetBufferPointer();
	paste_slice_info.interpolationMode = interpolation_strategy;
	paste_slice_info.compoundingMode = compounding_strategy;
	paste_slice_info.accOverflowCount = &accOverflow;
	paste_slice_info.pixelRejectionThreshold = 0;
	paste_slice_info.image_number = 0;

	Eigen::Matrix4d ref_to_image;
	ref_to_image(3, 0) = 0.0;
	ref_to_image(3, 1) = 0.0;
	ref_to_image(3, 2) = 0.0;
	ref_to_image(3, 3) = 1.0;

	std::vector<input_type::Pointer> local_image_copies;
	{
		std::lock_guard<std::mutex> g{mut};
		local_image_copies = std::move(frame_data);
		frame_data = std::vector<input_type::Pointer>();
		if(local_image_copies.size()==0)
			return false;
	}

	for (auto img : local_image_copies) {	
	    int inputFrameExtentForCurrentThread[6] = { 0, 0, 0, 0, 0, 0 };
		double clipRectangleOrigin [2]; // array size 2
		double clipRectangleSize [2]; // array size 2

		auto local_size = img->GetLargestPossibleRegion().GetSize();
		auto local_origin = img->GetOrigin();

		if(clipping){
			clipRectangleOrigin[0] = (*clipping).clipRectangleOrigin[0];
			clipRectangleOrigin[1] = (*clipping).clipRectangleOrigin[1];
			clipRectangleSize[0] = (*clipping).clipRectangleSize[0];
			clipRectangleSize[1] = (*clipping).clipRectangleSize[1];
		} else {
			clipRectangleOrigin[0] = 0;
			clipRectangleOrigin[1] = 0;
			clipRectangleSize[0] = local_size.GetSize()[0]-1;
			clipRectangleSize[1] = local_size.GetSize()[1]-1;
		}

		inputFrameExtentForCurrentThread[1] = local_size.GetSize()[0]-1;
		inputFrameExtentForCurrentThread[3] = local_size.GetSize()[1]-1;

		paste_slice_info.clipRectangleOrigin = clipRectangleOrigin;
	    paste_slice_info.clipRectangleSize = clipRectangleSize;
		paste_slice_info.inExt = inputFrameExtentForCurrentThread;
		paste_slice_info.image_number += 1;

		itk::Matrix<double> image_orientation = img->GetDirection();
		itk::Point<double> image_origin = img->GetOrigin();

		ref_to_image(0, 0) = image_orientation[0][0];
		ref_to_image(1, 0) = image_orientation[1][0];
		ref_to_image(2, 0) = image_orientation[2][0];

		ref_to_image(0, 1) = image_orientation[0][1];
		ref_to_image(1, 1) = image_orientation[1][1];
		ref_to_image(2, 1) = image_orientation[2][1];

		ref_to_image(0, 2) = image_orientation[0][2];
		ref_to_image(1, 2) = image_orientation[1][2];
		ref_to_image(2, 2) = image_orientation[2][2];

		ref_to_image(0, 3) = image_origin[0];
		ref_to_image(1, 3) = image_origin[1];
		ref_to_image(2, 3) = image_origin[2];

		// The matrix is the transformation of the 
		// origin of the output volume (1) to the 
		// origin of the input image (2). This is 
		// given by T02=T01*T12, and by premultiplying 
		// by T10=inverse(T01) we obtain T12=inverse(T01)*T02
		Eigen::Matrix4d output_to_origin = output_to_ref * ref_to_image;

        auto size_in = img->GetLargestPossibleRegion().GetSize();
        auto origin_in = img->GetOrigin();
        auto spacing_in = img->GetSpacing();
		paste_slice_info.inPtr = img->GetBufferPointer();
        paste_slice_info.in_origin[0] = origin_in[0];
        paste_slice_info.in_origin[1] = origin_in[1];
        paste_slice_info.in_origin[2] = origin_in[2];
        paste_slice_info.in_size[0] = size_in[0];
        paste_slice_info.in_size[1] = size_in[1];
        paste_slice_info.in_size[2] = size_in[2];
        paste_slice_info.in_spacing[0] = spacing_in[0];
        paste_slice_info.in_spacing[1] = spacing_in[1];
        paste_slice_info.in_spacing[2] = spacing_in[2];
		paste_slice_info.matrix = output_to_origin;

		// now we need to divide the image between equal patches, for that we copy the IGSIO 
		// block of code 
		std::vector<std::array<int,6>> block_divisions;
		block_divisions.resize(pool->size());
		if(!splice_input_extent(block_divisions,inputFrameExtentForCurrentThread))
			throw std::runtime_error("failure to execute slicing of input image");

		std::condition_variable cv;
		std::mutex local_mut;
		std::unique_lock<std::mutex> unique_{local_mut};
		int executed = 0;
		size_t index = 0;
		for(const auto& range : block_divisions){
			curan::utilities::Job job;
			job.description = "partial volume reconstruction";
			job.function_to_execute = [index,range,paste_slice_info,&executed,&local_mut,&cv](){
				size_t local_index = index;
				try{
					int this_thread_extent[6];
					std::memcpy(this_thread_extent,range.data(),6*sizeof(int));

					curan::image::reconstruction::PasteSliceIntoVolumeInsertSliceParamsTemplated<input_pixel_type,output_pixel_type> local_paste_slice_info;
					local_paste_slice_info = paste_slice_info;
					local_paste_slice_info.inExt = this_thread_extent;

					curan::image::reconstruction::TemplatedUnoptimizedInsertSlice<input_pixel_type,output_pixel_type,255>(&local_paste_slice_info);
					{
						std::lock_guard<std::mutex> g{local_mut};
						++executed;
						//std::printf("finished patch of work: %d (to do: %d)\n",executed,(int)block_divisions.size());
					}
					cv.notify_one();
				} catch(std::exception & e){
					std::cout << "exception was thrown in index :" << local_index << " with error message: " << e.what() << std::endl;
				}
			};
			++index;
			pool->submit(std::move(job));
		}
		//this blocks until all threads have processed their corresponding block that they need to process
		cv.wait(unique_,[&](){ return executed==block_divisions.size();});
	};
    textureData->dirty();
    return true;
}

}
}