from PIL import ImageFont
import visualkeras
import os
from tensorflow.keras.models import load_model


def Vis_CNN(model):
    font = ImageFont.truetype("arial.ttf", 24)  # using comic sans is strictly prohibited!
    model.add(visualkeras.SpacingDummyLayer(spacing=100))
    visualkeras.layered_view(model, to_file='self_driving_car_pkg/self_driving_car_pkg/data/Vis_CNN.png',legend=True, font=font,scale_z=2).show()  # font is optional!


def main():

    model = load_model(os.path.abspath('self_driving_car_pkg/self_driving_car_pkg/data/saved_model_5_Sign.h5'),compile=False)
    Vis_CNN(model)

if __name__ == '__main__':
	main()