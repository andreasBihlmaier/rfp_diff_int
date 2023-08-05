from setuptools import setup

package_name = 'rfp_diff_int'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahb',
    maintainer_email='andreas.bihlmaier@gmx.de',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj1d_generator = rfp_diff_int.traj1d_generator:main',
            'turtle_teleporter = rfp_diff_int.turtle_teleporter:main',
            'traj1d_differentiator = rfp_diff_int.traj1d_differentiator:main',
            'twist_integrator = rfp_diff_int.twist_integrator:main',
        ],
    },
)
