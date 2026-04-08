from setuptools import setup

package_name = 'arha_robotics_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/picking_objects.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kenneth',
    maintainer_email='user@todo.todo',
    description='Experiments and high-level Python scripts for ARHA, including the Teach Pendant GUI.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_pendant = arha_robotics_experiments.teach_pendant:main',
            'gravity_metrics = arha_robotics_experiments.gravity_metrics:main',
            'iso_path_accuracy = arha_robotics_experiments.iso_path_accuracy:main',
            'tc05_repeatability = arha_robotics_experiments.tc05_repeatability:main',
            'tc04_velocity_limit = arha_robotics_experiments.tc04_velocity_limit:main',
            'dual_camera_picker = arha_robotics_experiments.dual_camera_picker:main',
        ],
    },
)
