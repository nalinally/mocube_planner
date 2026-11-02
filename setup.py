from glob import glob

from setuptools import find_packages, setup

package_name = 'mocube_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/moves', glob('moves/*.json')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='obamanarito',
    maintainer_email='obama1632@gmail.com',
    description='Reconfiguration planning and plan execution for mocube modules',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plan = mocube_planner.plan:main',
            'render_plan = mocube_planner.render_plan:main',
            'plan_executor = mocube_planner.plan_executor:main',
            'sim_modules = mocube_planner.sim_modules:main',
        ],
    },
)
