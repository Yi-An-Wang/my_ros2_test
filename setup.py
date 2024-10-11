from setuptools import find_packages, setup

package_name = 'my_ros2_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yi-an',
    maintainer_email='wangia@solab.me.ntu.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moved_circle = my_ros2_test.moved_circle:main",
            "keyboard_control = my_ros2_test.keyboard_control:main",
            "obstaclesim1 = my_ros2_test.obstaclesim1:main",
            "GBM_plot = my_ros2_test.GBM_plot:main",
            "GBMsim = my_ros2_test.GBMsim:main",
            "GBM_keyboard = my_ros2_test.GBM_keyboard:main",
            "test_subscriber = my_ros2_test.test_subscriber:main",
            "GBM_tk = my_ros2_test.GBM_tk:main"
        ],
    },
)
