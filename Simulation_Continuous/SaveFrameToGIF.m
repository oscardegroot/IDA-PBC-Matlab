function SaveFrameToGIF(filename, fig, dt, index)
% Capture the plot as an image 
frame = getframe(fig); 
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
if (index == 1)
  imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', dt); 
else 
  imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', dt); 
end 