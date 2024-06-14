"""
CODE BASED ON EXAMPLE FROM:
@misc{coumans2017pybullet,
  title={Pybullet, a python module for physics simulation in robotics, games and machine learning},
  author={Coumans, Erwin and Bai, Yunfei},
  url={www.pybullet.org},
  year={2017},
}

Example: heightfield.py
https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/heightfield.py
"""

import pybullet_data as pd
import random

# Constants defining different sources of heightfield
useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2

# Parameters for heightfield generation
numHeightfieldRows = 256
numHeightfieldColumns = 256
heightfieldSource = useProgrammatic
random.seed(10)  # Set random seed for reproducibility


class HeightField:
    def __init__(self):
        self.hf_id = 0
        self.terrainShape = 0
        self.heightfieldData = [0] * numHeightfieldRows * numHeightfieldColumns

    def _generate_field(self, env, heightPerturbationRange=0.08):
        """
        Generate the heightfield terrain based on the chosen source.

        Args:
            env (object): The environment object with pybullet client.
            heightPerturbationRange (float): Range of height perturbation.

        Notes:
            This method generates a heightfield terrain based on the selected source.
            It uses the pybullet client from the environment to create and configure
            the heightfield terrain accordingly.
        """
        env.pybullet_client.setAdditionalSearchPath(pd.getDataPath())

        env.pybullet_client.configureDebugVisualizer(
            env.pybullet_client.COV_ENABLE_RENDERING, 0)

        if heightfieldSource == useProgrammatic:
            # Generate heightfield programmatically
            for j in range(int(numHeightfieldColumns / 2)):
                for i in range(int(numHeightfieldRows / 2)):
                    height = random.uniform(0, heightPerturbationRange)
                    # Set heights for each quadrant of the heightfield
                    self.heightfieldData[2 * i +
                                         2 * j * numHeightfieldRows] = height
                    self.heightfieldData[2 * i + 1 +
                                         2 * j * numHeightfieldRows] = height
                    self.heightfieldData[2 * i + (2 * j + 1) *
                                         numHeightfieldRows] = height
                    self.heightfieldData[2 * i + 1 + (2 * j + 1) *
                                         numHeightfieldRows] = height

            # Create the collision shape for the heightfield
            terrainShape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.07, .07, 1.6],  # Scale of the terrain mesh
                heightfieldTextureScaling=(numHeightfieldRows - 1) / 2,
                heightfieldData=self.heightfieldData,
                numHeightfieldRows=numHeightfieldRows,
                numHeightfieldColumns=numHeightfieldColumns)

            # Create the multi-body for the terrain
            terrain = env.pybullet_client.createMultiBody(0, terrainShape)

            # Reset position and orientation of the terrain
            env.pybullet_client.resetBasePositionAndOrientation(
                terrain, [0, 0, 0.0], [0, 0, 0, 1])

            # Adjust dynamics parameters (friction)
            env.pybullet_client.changeDynamics(terrain, -1, lateralFriction=1.0)

        if heightfieldSource == useDeepLocoCSV:
            # Example for loading heightfield from a CSV file (not implemented here)

            # Replace with actual file path and parameters
            terrainShape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.5, .5, 2.5],
                fileName="heightmaps/ground0.txt",  # Example file path
                heightfieldTextureScaling=128)

            # Create the multi-body for the terrain
            terrain = env.pybullet_client.createMultiBody(0, terrainShape)

            # Reset position and orientation of the terrain
            env.pybullet_client.resetBasePositionAndOrientation(
                terrain, [0, 0, 0], [0, 0, 0, 1])

            # Adjust dynamics parameters (friction)
            env.pybullet_client.changeDynamics(terrain, -1, lateralFriction=1.0)

        if heightfieldSource == useTerrainFromPNG:
            # Example for loading heightfield and texture from PNG files (not implemented here)

            # Replace with actual file paths and parameters
            terrainShape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.05, .05, 1.8],
                fileName="heightmaps/wm_height_out.png")  # Example file path

            textureId = env.pybullet_client.loadTexture(
                "heightmaps/gimp_overlay_out.png")  # Example texture path

            # Create the multi-body for the terrain
            terrain = env.pybullet_client.createMultiBody(0, terrainShape)

            # Change visual shape to apply texture
            env.pybullet_client.changeVisualShape(terrain, -1, textureUniqueId=textureId)

            # Reset position and orientation of the terrain
            env.pybullet_client.resetBasePositionAndOrientation(
                terrain, [0, 0, 0.1], [0, 0, 0, 1])

            # Adjust dynamics parameters (friction)
            env.pybullet_client.changeDynamics(terrain, -1, lateralFriction=1.0)

        self.hf_id = terrainShape
        self.terrainShape = terrainShape
        print("TERRAIN SHAPE: {}".format(terrainShape))

        # Change visual shape color to w
