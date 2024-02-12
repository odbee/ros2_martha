from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'mv_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='eddysjustfortrash@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mp_scene = mv_py.mp_scene:main',
            'mp_api = mv_py.mp_api:main',
            'mp_api_2 = mv_py.mp_api_2:main',
            'mp_client = mv_py.mp_client:main'

        ],
    },
)
