from setuptools import find_packages, setup

package_name = 'dsr_example'

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
    maintainer='gossi',
    maintainer_email='mincheol710313@gmail.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dance = dsr_example.demo.dance_m1013:main',
                'single_robot_simple = dsr_example.simple.single_robot_simple:main',
                'test = dsr_example.project.test:main',
                'pick_and_place = dsr_example.project.01_pick_and_place:main',
                'project = dsr_example.project.project:main',
                'block = dsr_example.project.block:main',
                'block_sort = dsr_example.project.02_block_sort:main',
                'shelf_loading = dsr_example.project.03_shelf_loading:main',
        ],
    },
)
