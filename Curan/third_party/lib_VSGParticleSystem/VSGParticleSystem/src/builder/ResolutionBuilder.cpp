#include <vsgParticleSystem/builder/ResolutionBuilder.h>

using namespace vsg;

ref_ptr<Node> ResolutionBuilder::createSphere(const GeometryInfo &info,
                                              const StateInfo &stateInfo,
                                              const size_t num_columns,
                                              const size_t num_rows) {
    auto &subgraph = _spheres[info];
    if (subgraph)
        return subgraph;

    uint32_t instanceCount = 1;
    auto positions = instancePositions(info, instanceCount);
    auto colors = instanceColors(info, instanceCount);
    auto [t_origin, t_scale, t_top] = y_texcoord(stateInfo).value;

    // create StateGroup as the root of the scene/command graph to hold the
    // GraphicsProgram, and binding of Descriptors to decorate the whole graph
    auto scenegraph = createStateGroup(stateInfo);

    auto dx = info.dx * 0.5f;
    auto dy = info.dy * 0.5f;
    auto dz = info.dz * 0.5f;
    auto origin = info.position;

    unsigned int num_vertices = num_columns * num_rows;
    unsigned int num_indices = (num_columns - 1) * (num_rows - 1) * 6;

    auto vertices = vec3Array::create(num_vertices);
    auto normals = vec3Array::create(num_vertices);
    auto texcoords = vec2Array::create(num_vertices);
    auto indices = ushortArray::create(num_indices);

    for (unsigned int r = 0; r < num_rows; ++r) {
        float beta = ((float(r) / float(num_rows - 1)) - 0.5f) * PIf;
        float ty = t_origin + t_scale * float(r) / float(num_rows - 1);
        float cos_beta = cosf(beta);
        vec3 dz_sin_beta = dz * sinf(beta);

        vec3 v = dy * cos_beta + dz_sin_beta;
        vec3 n = normalize(v);

        unsigned int left_i = r * num_columns;
        vertices->set(left_i, v + origin);
        normals->set(left_i, n);
        texcoords->set(left_i, vec2(0.0f, ty));

        unsigned int right_i = left_i + num_columns - 1;
        vertices->set(right_i, v + origin);
        normals->set(right_i, n);
        texcoords->set(right_i, vec2(1.0f, ty));

        for (unsigned int c = 1; c < num_columns - 1; ++c) {
            unsigned int vi = left_i + c;
            float alpha = (float(c) / float(num_columns - 1)) * 2.0f * PIf;
            v = dx * (-sinf(alpha) * cos_beta) + dy * (cosf(alpha) * cos_beta) +
                dz_sin_beta;
            n = normalize(v);
            vertices->set(vi, origin + v);
            normals->set(vi, n);
            texcoords->set(vi, vec2(float(c) / float(num_columns - 1), ty));
        }
    }

    unsigned int i = 0;
    for (unsigned int r = 0; r < num_rows - 1; ++r) {
        for (unsigned int c = 0; c < num_columns - 1; ++c) {
            unsigned lower = num_columns * r + c;
            unsigned upper = lower + num_columns;

            indices->set(i++, lower);
            indices->set(i++, lower + 1);
            indices->set(i++, upper);

            indices->set(i++, upper);
            indices->set(i++, lower + 1);
            indices->set(i++, upper + 1);
        }
    }

    if (info.transform != identity) {
        transform(info.transform, vertices, normals);
    }

    // setup geometry
    auto vid = VertexIndexDraw::create();

    DataList arrays;
    arrays.push_back(vertices);
    if (normals)
        arrays.push_back(normals);
    if (texcoords)
        arrays.push_back(texcoords);
    if (colors)
        arrays.push_back(colors);
    if (positions)
        arrays.push_back(positions);
    vid->assignArrays(arrays);

    vid->assignIndices(indices);
    vid->indexCount = static_cast<uint32_t>(indices->size());
    vid->instanceCount = instanceCount;

    scenegraph->addChild(vid);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    subgraph = scenegraph;
    return subgraph;
}