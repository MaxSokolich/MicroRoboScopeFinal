import imageio
import os

def save_gif_from_frames(folder="demo_frames", gif_name="trajectory.gif", fps=30):
    images = []
    files = sorted([f for f in os.listdir(folder) if f.endswith(".png")])
    for filename in files:
        img_path = os.path.join(folder, filename)
        images.append(imageio.imread(img_path))
    imageio.mimsave(gif_name, images, fps=fps)
    print(f"GIF saved to {gif_name}")

save_gif_from_frames(folder="demo_frames", gif_name="trajectory.gif", fps=10)