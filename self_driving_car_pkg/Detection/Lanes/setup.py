from setuptools import setup
import os
from glob import glob


root_package = 'prius_sdc'
package_name = 'prius_sdc'
det_l_module ="prius_sdc/Detection/Lanes"
det_s_module ="prius_sdc/Detection/Signs"
config_module = "prius_sdc/config" 
data_module ="prius_sdc/data"
control_module ="prius_sdc/Control"


    
setup(
    name=root_package,
    version='0.0.0',
    packages=[package_name,config_module,det_l_module,det_s_module,data_module,control_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + root_package]),
        ('share/' + root_package, ['package.xml']),
        (os.path.join('share', root_package), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'sdc = prius_sdc.computer_vision_node:main',

        ],
    },
)
