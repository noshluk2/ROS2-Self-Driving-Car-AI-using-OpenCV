from PIL import ImageFont
#import visualkeras
#import cv2
import os
#from Detection.Signs.b_Classification.Visualize_CNN import Vis_CNN
from tensorflow.keras.models import load_model
from quiver_engine import server

def Vis_CNN(model):
    font = ImageFont.truetype("arial.ttf", 24)  # using comic sans is strictly prohibited!
    model.add(visualkeras.SpacingDummyLayer(spacing=100))
    visualkeras.layered_view(model, to_file='data/Vis_CNN.png',legend=True, font=font,scale_z=2).show()  # font is optional!


def main():

    model = load_model(os.path.abspath('data/saved_model.h5'),compile=False)
    #Vis_CNN(model)
    server.launch(model, classes=['speed_sign_70','speed_sign_80','stop','No_Sign'], input_folder='Detection/Signs/b_Classification/test_images')

if __name__ == '__main__':
	main()