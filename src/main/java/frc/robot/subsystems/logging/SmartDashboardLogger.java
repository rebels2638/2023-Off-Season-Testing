// package frc.robot.subsystems.logging;

//  import java.util.Set;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class SmartDashboardLogger extends SubsystemBase {
//     public SmartDashboardLogger(){}

//     @Override
//     public void periodic() {
//         Set<String> setKeys = SmartDashboard.getKeys();
//         Object[] keys = setKeys.toArray();
//         for (int keyNum = 0; keyNum < setKeys.size(); keyNum++) {
//             NetworkTableEntry entry = SmartDashboard.getEntry( (String) keys[keyNum] );
//             NetworkTableType type = entry.getType(); 
//             switch(type) {
//                 case kBoolean:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getBoolean(false));
//                     break;
//                 case kDouble:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getDouble(0));
//                     break;
//                 case kString:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getString(""));
//                     break;
//                 case kBooleanArray:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getBooleanArray(new boolean[0]));
//                     break;
//                 case kDoubleArray:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getDoubleArray(new double[0]));
//                     break;
//                 case kStringArray:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getStringArray(new String[0]));
//                     break;
//                 case kRaw:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getRaw(new byte[0]));
//                     break;
//                 case kFloat:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getFloat(0));
//                     break;
//                 case kFloatArray:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getFloatArray(new float[0]));
//                     break;
//                 case kIntegerArray:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getIntegerArray(new long[0]));
//                     break;
//                 case kInteger:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], (long) entry.getNumber(0));
//                     break;
//                 default:
//                     Logger.getInstance().recordOutput((String) keys[keyNum], entry.getValue().toString());
//                     break;
                
//             }

            
//         }
//     }
// }