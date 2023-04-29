function ReProj = reproj_mat(ProjLeft, ProjRight)
%Create the reprojection matrix from the individual projection matrices
%reprojectionMatrix = [1 0 0 -cx; 0 1 0 -cy; 0 0 0 f; 0 0 1/b 0];
    ReProj = [1 0 0 -ProjLeft(1,3);0 1 0 -ProjLeft(2,3); 0 0 0 ProjLeft(1,1); 0 0 -ProjRight(1,1)/ProjRight(1,4) (ProjLeft(1,3)-ProjRight(1,3))*ProjRight(1,1)/ProjRight(1,4)];
end