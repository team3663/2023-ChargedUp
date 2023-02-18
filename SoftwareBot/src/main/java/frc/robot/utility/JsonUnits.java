package frc.robot.utility;

import com.fasterxml.jackson.annotation.JacksonAnnotationsInside;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.deser.ContextualDeserializer;
import com.fasterxml.jackson.databind.ser.ContextualSerializer;
import edu.wpi.first.math.util.Units;

import java.io.IOException;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@JsonSerialize(using = UnitSerializer.class)
@JsonDeserialize(using = UnitDeserializer.class)
@JacksonAnnotationsInside
@Retention(RetentionPolicy.RUNTIME)
public @interface JsonUnits {
    Unit value();

    enum Unit {METERS, INCHES, FEET, RADIANS, DEGREES, ROTATIONS}
}

class UnitSerializer extends JsonSerializer<Double> implements ContextualSerializer {
    private final JsonUnits.Unit unit;

    public UnitSerializer() {
        this(JsonUnits.Unit.METERS);
    }

    private UnitSerializer(JsonUnits.Unit unit) {
        this.unit = unit;
    }

    @Override
    public void serialize(Double value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
        String stringValue = null;

        switch (unit) {
            case METERS:
                stringValue = value + " meters";
                break;
            case INCHES:
                stringValue = Units.metersToInches(value) + " inches";
                break;
            case FEET:
                stringValue = Units.metersToFeet(value) + " feet";
                break;
            case RADIANS:
                stringValue = value + " radians";
                break;
            case DEGREES:
                stringValue = Units.radiansToDegrees(value) + " degrees";
                break;
            case ROTATIONS:
                stringValue = Units.radiansToRotations(value) + " rotations";
                break;
        }

        gen.writeString(stringValue);
    }

    @Override
    public JsonSerializer<?> createContextual(SerializerProvider prov, BeanProperty property) throws JsonMappingException {
        var units = property.getAnnotation(JsonUnits.class);

        return new UnitSerializer(units.value());
    }
}

class UnitDeserializer extends JsonDeserializer<Double> implements ContextualDeserializer {

    @Override
    public Double deserialize(JsonParser p, DeserializationContext ctxt) throws IOException, JsonProcessingException {
        return 0.0;
    }

    @Override
    public JsonDeserializer<?> createContextual(DeserializationContext ctxt, BeanProperty property) throws JsonMappingException {
        return this;
    }
}