extern "C"
void integrate(float * rgba,float * cam_K, float * cam2base, float * depth_im,
int im_height, int im_width, int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z, float voxel_size, float trunc_margin,
               float * voxel_grid_TSDF, float * voxel_grid_weight,float * voxel_grid_rgb);
//void integrate(float * cam_K, float * cam2base, float * depth_im,
//             int im_height, int im_width, int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
//               float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z, float voxel_size, float trunc_margin,
//               float * voxel_grid_TSDF, float * voxel_grid_weight);
