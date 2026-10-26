from setuptools import setup

package_name = 'apriltag_localization_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Adeel Zahid',
    maintainer_email='adeel.m.zahid@gmail.com',
    description='Calibration, tag generation and accuracy analysis utilities.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_apriltag_textures = apriltag_localization_tools.generate_apriltag_textures:main',
            'calibrate_camera = apriltag_localization_tools.calibrate_camera:main',
            'localization_error_analysis = apriltag_localization_tools.localization_error_analysis:main',
        ],
    },
)
