variable "REGISTRY" {
  default = "docker.io"
}

variable "IMAGE_NAME" {
  default = "robodk-crx-kinematics"
}

variable "CRX_KINEMATICS_VERSION" {
  default = "0.3.1"
}

variable "ROBODK_VERSION" {
  default = "6.0.6"
}

variable "ROBODK_DOWNLOAD_SHA256" {
  default = "0d965236db076d35f9337a93b3ee79dc75a361383f63ca230f33bd2658b55701"
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
    ROBODK_DOWNLOAD_SHA256  = ROBODK_DOWNLOAD_SHA256
    ROBODK_VERSION          = ROBODK_VERSION
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
