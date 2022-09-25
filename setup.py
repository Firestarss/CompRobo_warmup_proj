from setuptools import setup

package_name = 'comprobo_warmup_proj'

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
    maintainer='firestarss',
    maintainer_email='firestarss88@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = comprobo_warmup_proj.teleop:main',
            'drive_square = comprobo_warmup_proj.drive_square:main',
            'wall_follower = comprobo_warmup_proj.wall_follower:main',
            'obstacle_avoider = comprobo_warmup_proj.obstacle_avoider:main',
            'person_follower = comprobo_warmup_proj.person_follower:main',
            'finite_state_controller = comprobo_warmup_proj.finite_state_controller:main'
        ],
    },
)
