#include <memory>

constexpr size_t number_of_vals_per_vertex = 3;
constexpr size_t triangles_per_quad = 2;
constexpr size_t indices_per_triangle = 3;

size_t getNumberOfTriangles(size_t width, size_t height){
    return width*height*triangles_per_quad;
}

size_t getVerticesCount( size_t width, size_t height ) {
    return (width+1) * (height+1) * number_of_vals_per_vertex;
}

size_t getIndicesCount( size_t width, size_t height ) {
    return getNumberOfTriangles(width,height)*indices_per_triangle;
}

std::unique_ptr<float[]> getVertices( size_t width, size_t height ) {
    std::unique_ptr<float[]> vertices = std::make_unique<float[]>(getVerticesCount( width, height ));
    int i = 0;
    for ( int row=0; row<height; row++ ) {
        for ( int col=0; col<width; col++ ) {
            vertices[i++] = (float) col;
            vertices[i++] = 0.0f;
            vertices[i++] = (float) row;
        }
    }
    return vertices;
}

std::unique_ptr<size_t[]> getIndices( size_t width, size_t height ) {
    std::unique_ptr<size_t[]> indices = std::make_unique<size_t[]>(getIndicesCount(width,height));
    size_t index = 0;
    for ( int row=0; row<height; ++row ) {
        for ( int col=0; col< width; ++col ) {
                indices[index++] = col + row * width; //first vertex
                indices[index++] = col + (row+1) * width; //second vertex
                indices[index++] = col + (row+1) * width; //third vertex
            }
    }
    return indices;
}

int main(){
    size_t width = 3;
    size_t height = 3;
    auto vertices = getVertices(width,height);
    auto indices = getIndices(width,height);

    return 0;
}