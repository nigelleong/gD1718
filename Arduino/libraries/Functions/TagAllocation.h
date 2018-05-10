
void tag_to_position(float * tag, String * str){
	
	float x_pos, y_pos;
	
	switch(str){
		case "04D5C4BA625A80": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04CDC4BA625A80": 
		 x_pos = 250; 
		 y_pos = 500; 
		 break; 

		case "04C6C3BA625A80": 
		 x_pos = 250; 
		 y_pos = 750; 
		 break; 

		case "04BFC2BA625A80": 
		 x_pos = 250; 
		 y_pos = 1000; 
		 break; 

		case "04B7C2BA625A80": 
		 x_pos = 250; 
		 y_pos = 1250; 
		 break; 

		case "04F4C4BA625A80": 
		 x_pos = 250; 
		 y_pos = 1500; 
		 break; 

		case "04FCC4BA625A80": 
		 x_pos = 250; 
		 y_pos = 1750; 
		 break; 

		case "0404C4BA625A81": 
		 x_pos = 250; 
		 y_pos = 2000; 
		 break; 

		case "040CC4BA625A81": 
		 x_pos = 250; 
		 y_pos = 2250; 
		 break; 

		case "0414C4BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0404C5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "040CC5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0414C5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "041CC5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0424C5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "044CC5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0444C5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "043CC5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0434C5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "042CC5BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0462C6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "045AC6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0454C4BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "044CC4BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0444C4BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0472C7BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "046AC7BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0462C7BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "045AC7BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0453C6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "047CC7BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0483C6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "047BC6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0472C6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "046AC6BA625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04FC0BC2625A80": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04010AC2625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04080BC2625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "040E0DC2625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04150EC2625A81": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04B3C6BA625A80": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04AAC6BA625A80": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "04A2C6BA625A80": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 

		case "0498C6BA625A80": 
		 x_pos = 250; 
		 y_pos = 250; 
		 break; 
		 
		default:
		 Serial.println("Invalid tag UID");
		 break;

	}
tag[0] = x_pos;
tag[1] = y_pos;
}
	
