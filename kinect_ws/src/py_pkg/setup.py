from setuptools import find_packages, setup

package_name = 'py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),



        ('share/' + package_name +'/launch', ['py_pkg/launch/ejecutar.py']), # aniadido para el launch




    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leyla',
    maintainer_email='leyla@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'pub= py_pkg.pub:main',
            'd_aruco= py_pkg.d_aruco:main',

            'd_vel= py_pkg.d_vel:main',
            'servo= py_pkg.servo:main',
            'lidar= py_pkg.lidar:main',

            
            'trans= py_pkg.trans:main',



        ],
    },
)


