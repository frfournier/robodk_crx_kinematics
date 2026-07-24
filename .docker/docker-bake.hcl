variable "REGISTRY" {
  default = "docker.io"
}

variable "IMAGE_NAME" {
  default = "robodk-crx-kinematics"
}

variable "CRX_KINEMATICS_VERSION" {
  default = "0.3.1"
}

group "default" {
  targets = ["robodk-crx-kinematics"]
}

target "robodk-crx-kinematics" {
  context    = ".."
  dockerfile = ".docker/Dockerfile"
  target     = "runtime"
  platforms  = ["linux/amd64"]

  args = {
    CRX_KINEMATICS_VERSION = CRX_KINEMATICS_VERSION
    ROBODK_ACCEPT_EULA      = "yes"
  }

  tags = [
    "${REGISTRY}/${IMAGE_NAME}:${CRX_KINEMATICS_VERSION}",
    "${REGISTRY}/${IMAGE_NAME}:latest",
  ]

  attest = [
    "type=provenance,mode=max",
    "type=sbom",
  ]
}
