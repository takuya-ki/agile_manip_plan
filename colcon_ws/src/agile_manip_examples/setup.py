from setuptools import find_packages, setup

package_name = 'agile_manip_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/graspgen_demo.launch.py',
             'launch/cumotion_demo.launch.py',
             'launch/grasp_and_motion_demo.launch.py',
             'launch/obstacle_aware_demo.launch.py',
             'launch/benchmark.launch.py']),
        ('share/' + package_name + '/config',
            ['config/graspgen_antipodal.yaml',
             'config/graspgen_suction.yaml',
             'config/cumotion_iiwa14.yaml',
             'config/grasp_and_motion.yaml',
             'config/obstacle_aware.yaml',
             'config/benchmark.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takuya Kiyokawa',
    maintainer_email='taku8926@gmail.com',
    description='Example programs for agile manipulation planning.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'graspgen_client = agile_manip_examples.graspgen_client:main',
            'cumotion_client = agile_manip_examples.cumotion_client:main',
            'grasp_and_motion_planner = agile_manip_examples.grasp_and_motion_planner:main',
            'obstacle_aware_grasp_and_motion_planner = agile_manip_examples.obstacle_aware_grasp_and_motion_planner:main',
            'benchmark_harness = agile_manip_examples.benchmark_harness:main',
        ],
    },
)
