function [images, depth] = read_im_from_run(run_folder_name, num_photo_taken)
    depth_path_name = run_folder_name + "\depth\image";
    image_path_name = run_folder_name + "\images\image";

    images = zeros(768,1024,num_photo_taken*3);
    depth = zeros(768,1024,num_photo_taken*3);

    for i = 0:num_photo_taken-1
        inter_d = im2double(imread(depth_path_name + sprintf("%06d",i) + "_disp_color.png"));
        depth(:,:,(3*i+1):3*(i+1)) = inter_d(:,1:1024,:);
        images(:,:, (3*i+1):3*(i+1)) = im2double(imread(image_path_name + sprintf("%06d",i) + "_left.png"));
    end
    %disparityRaw = imread("Run 2\depth\image000000_disp_raw.png"); 
    %frameLeftRect = imread("Run 2\images\image000000_left.png");
    %frameRightRect = imread("Run 2\images\image00000#_right.png");

end