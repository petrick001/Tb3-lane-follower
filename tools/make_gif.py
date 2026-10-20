"""Convert the demo MP4 videos in media/ into small GIFs.

GitHub shows GIFs directly inside a README, but it does not play MP4 files from the repo.
Usage (from the repo folder):  python3 tools/make_gif.py
"""
import cv2
from PIL import Image

# ---- Settings you can change -------------------------------------------------
OUTPUT_WIDTH = 480    # width of the GIF in pixels (smaller = lighter file)
GIF_FPS = 8           # frames per second in the GIF (smaller = lighter file)
MAX_SECONDS = 15      # only the first 15 seconds of the video are used

# ---- Which videos to convert: (input mp4, output gif) ------------------------
VIDEOS = [
    ("media/part1_robot_on_track.mp4", "media/part1_robot_on_track.gif"),
    ("media/part1_rqt_view.mp4", "media/part1_rqt_view.gif"),
]


def video_to_gif(video_path, gif_path):
    """Read a video, keep a few frames per second, shrink them, and save as a GIF."""
    video = cv2.VideoCapture(video_path)
    if not video.isOpened():
        print("Cannot open", video_path)
        return

    # Frames per second of the original video (use 30 if the file does not say).
    video_fps = video.get(cv2.CAP_PROP_FPS)
    if video_fps <= 0:
        video_fps = 30

    # Keep one frame out of every `step` frames, to reach about GIF_FPS.
    step = max(1, round(video_fps / GIF_FPS))
    max_frames = int(MAX_SECONDS * video_fps)

    frames = []
    frame_number = 0
    while frame_number < max_frames:
        ok, frame = video.read()
        if not ok:          # end of the video
            break
        if frame_number % step == 0:
            # Shrink the frame, keeping the same shape.
            height, width = frame.shape[:2]
            new_height = int(height * OUTPUT_WIDTH / width)
            small = cv2.resize(frame, (OUTPUT_WIDTH, new_height))

            # OpenCV uses BGR colours, Pillow uses RGB: convert.
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)

            # Fewer colours = smaller GIF file.
            image = Image.fromarray(rgb).convert("P", palette=Image.ADAPTIVE, colors=128)
            frames.append(image)
        frame_number += 1
    video.release()

    if not frames:
        print("No frames read from", video_path)
        return

    # Save all frames as one looping GIF.
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / GIF_FPS),   # milliseconds per frame
        loop=0,                          # 0 = loop forever
        optimize=True,
    )
    print("Saved", gif_path, "with", len(frames), "frames")


for video_path, gif_path in VIDEOS:
    video_to_gif(video_path, gif_path)
