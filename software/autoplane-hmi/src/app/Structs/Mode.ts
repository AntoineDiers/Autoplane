export enum Mode {
  IDLE = "IDLE",
  TAKEOFF = "TAKEOFF",
  MANUAL = "MANUAL",
  AUTO = "AUTO",
  LANDING = "LANDING",
  EMERGENCY = "EMERGENCY"
}

export const DefaultMode : Mode = Mode.IDLE;